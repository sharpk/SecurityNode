// Security Node
// This sketch is for a JeeNode attached to a security system
// Copyright (c) 2012 Ken Sharp
// License: http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <EEPROM.h>

#define DEBUG	// comment this out to disable debug messages on serial port
#define NODEID 2
#define MASTER_NODEID 1
#define SENSOR_POLL_RATE 500  // in ms
#define KEYPAD_POLL_RATE 34  // in ms
#define SEND_THRESHOLD 4 // in ADC counts
#define CMD_BUFFER_MAX 10
#define OUT_BUFFER_MAX 4
#define PASSCODE_MAX 4
#define KEYPAD_CMD_TIMEOUT 5000 // in ms
#define DELAY_ARM_VALUE	30000	// in ms
#define MOTION_STARTUP_TIME  60000 // motion sensor has a "warm up" time after power on where sensor value should be ignored
#define SIREN_TIME_OUT 5	// in minutes
//#define MOTION_ALWAYS_ON 1	// enable this to start the motion sensor at startup, otherwise arming/disarming the system will turn it on or off
//#define SIREN_WHEN_ALARM 1	// enable this to turn on the siren when the alarm is triggered

#ifdef DEBUG
	#define DBG_PRINT(a) Serial.println(a)
	#define DBG_VAL(a) Serial.print(a)
#else
	#define DBG_PRINT(a)
	#define DBG_VAL(a)
#endif

// Message IDs
#define MSG_SENSOR 1
#define MSG_EMERGENCY 2
#define MSG_ARM 3
#define MSG_ALARM 4

// according to http://arduino.cc/en/Reference/Word
#define WORD_MAX 65535
#define ULONG_MAX 4294967295

// defines for DSC PC1500RK
#define PC1500RK_CLOCKPIN A4
#define PC1500RK_DATAPIN A5

#define NO_FLAGS 0x0000
#define BUZZER 0x0001
#define TROUBLE 0x0008
#define BYPASS 0x0010
#define MEMORY 0x0020
#define ARMED 0x0040
#define READY 0x0080
#define ZONE_1 0x8000
#define ZONE_2 0x4000
#define ZONE_3 0x2000
#define ZONE_4 0x1000
#define ZONE_5 0x0800
#define ZONE_6 0x0400

#define NO_KEY_PRESSED 0

byte sensor_update;
unsigned char outData[OUT_BUFFER_MAX];
MilliTimer sendTimer;
MilliTimer keypadTimer;
MilliTimer armTimer;
MilliTimer sirenTimer;
unsigned char sirenMinCnt;
Port port1(1);
Port port2(2);
Port port3(3);
Port port4(4);
unsigned char cmdBuf[CMD_BUFFER_MAX];
const char defaultPasscode[] = "1234";
char passcode[PASSCODE_MAX+1];
char emergency;
char armed;
char alarm;
char bypass_zone;
char motion_state;  // 0 - off, 1 - startup, 2 - active
unsigned long motion_start_time;

void setup() {
  int i;

#ifdef DEBUG
  Serial.begin(57600);
#endif
  
  rf12_initialize(NODEID, RF12_915MHZ, 212);

  port1.mode(INPUT);
  port1.mode2(OUTPUT);
  port2.mode(INPUT);
  port2.mode2(OUTPUT);
  port3.mode(INPUT);
  port3.mode2(INPUT);
  port4.mode(INPUT);
  port4.mode2(INPUT);
  
  // disable power to motion sensor and siren
  port1.digiWrite2(0);
  port2.digiWrite2(0);
  
  // enable pull-up resistors for digital inputs
  port1.digiWrite(1);
  port2.digiWrite(1);
  port3.digiWrite(1);
  port3.digiWrite2(1);
  port4.digiWrite(1);
  
  pinMode(PC1500RK_DATAPIN, OUTPUT);
  pinMode(PC1500RK_CLOCKPIN, OUTPUT);
  
  // load passcode from EEPROM, don't use addresses 0x20-0x40 since that's used by the rf12 driver
  // TODO: checksum the passcode in EEPROM
  passcode[PASSCODE_MAX] = 0; // NULL terminate passcode
  if (EEPROM.read(0x00) == 0xff) {
	//passcode is uninitialized
	memcpy(passcode, defaultPasscode, PASSCODE_MAX);
	DBG_PRINT("EEPROM unitialized; loading default passcode");
  }
  else {
	for (i = 0; i < PASSCODE_MAX; i++)
		passcode[i] = EEPROM.read(i);
	DBG_PRINT("EEPROM read; passcode=");
	DBG_PRINT(passcode);
  }
  
  emergency = 0;
  armed = 0;
  alarm = 0;
  bypass_zone = 0;
  motion_state = 0;
  motion_start_time = 0;
  
#ifdef MOTION_ALWAYS_ON
  motion_start();
  DBG_PRINT("In setup(): motion ON");
#endif
}

char PC1500RK_transfer(int control)
{
  int i,j=0,k=control;
  int bitcount=0;
  
  for(i=0; i<8; i++) {
    j <<= 1;
    digitalWrite(PC1500RK_DATAPIN, HIGH);
    digitalWrite(PC1500RK_CLOCKPIN, LOW);
    delayMicroseconds(625);
    if(digitalRead(PC1500RK_DATAPIN) == LOW)
      j |= 1;
    digitalWrite(PC1500RK_CLOCKPIN, HIGH);
    delayMicroseconds(625);
  }
  
  for(i=0; i<16; i++) {
    if(k & 0x8000)
      digitalWrite(PC1500RK_DATAPIN, HIGH);
    else
      digitalWrite(PC1500RK_DATAPIN, LOW);
    digitalWrite(PC1500RK_CLOCKPIN, LOW);
    delayMicroseconds(625);
    digitalWrite(PC1500RK_CLOCKPIN, HIGH);
    delayMicroseconds(625);
    k <<= 1;
  }
  
  switch (j) {
	case 0x41:
		return '1';
	case 0x42:
		return '4';
	case 0x44:
		return '7';
	case 0x48:
		return '*';
	case 0x21:
		return '2';
	case 0x22:
		return '5';
	case 0x24:
		return '8';
	case 0x28:
		return '0';
	case 0x11:
		return '3';
	case 0x12:
		return '6';
	case 0x14:
		return '9';
	case 0x18:
		return '#';
	case 0x90:
		return 'P';
	case 0xa0:
		return 'E';
	case 0xc0:
		return 'F';
	default:
		return NO_KEY_PRESSED;
  }
}

char cmd_timed_out(unsigned long cmd_start) {
	unsigned long time_elapsed;
	
	// handle timer rollover
	// TODO: change any calls to millis() to use MilliTimer instead (to simplify code)
	if (cmd_start <= millis())
		time_elapsed = millis() - cmd_start;
	else
		time_elapsed = (ULONG_MAX - cmd_start) + millis();
		
	if (time_elapsed > KEYPAD_CMD_TIMEOUT)
		return 1;
	else
		return 0;
}

void motion_start() {
	if (motion_state)
		return;
	else {
		port1.digiWrite2(1);
		motion_state = 1;
		motion_start_time = millis();
	}
}

void motion_stop() {
	port1.digiWrite2(0);
	motion_state = 0;
	motion_start_time = 0;
}

// check if motion sensor is done with startup
// 0 - still starting or disabled, 1 - active
char motion_good() {
	unsigned long time_elapsed;

	if (motion_state < 1)
		return 0;
	
	if (motion_state > 1)
		return 1;
		
	if (motion_start_time <= millis())
		time_elapsed = millis() - motion_start_time;
	else
		time_elapsed = (ULONG_MAX - motion_start_time) + millis();
		
	if (time_elapsed > MOTION_STARTUP_TIME) {
		motion_state = 2;
		return 1;
	}
	else
		return 0;
}

void siren_on() {
	port2.digiWrite2(1);
	sirenTimer.set(60000);  //set to a minute
	sirenMinCnt = SIREN_TIME_OUT - 1;
}

void siren_off() {
	port2.digiWrite2(0);
	sirenMinCnt = 0;
}

char siren_timeout() {
	if (sirenTimer.poll()) {
		if (!sirenMinCnt)
			return 1;
		sirenMinCnt--;
		sirenTimer.set(60000);
		return 0;
	}
	
	if (sirenTimer.remaining() || sirenMinCnt)
		return 0;
		
	return 1;
}

void keypad_statemachine() {
	char key;
	int i;
	static byte in_command;
	static unsigned int flags = READY;
	static unsigned long cmd_start = 0;
	static char last_key = NO_KEY_PRESSED;
	char new_key=0;
#ifdef DEBUG
	unsigned int last_flags;	
	last_flags = flags;
#endif
	
	// TODO: Add a "beep-beep-beep" from the buzzer when a door opens
	// front door
	if (port2.digiRead())
		flags |= ZONE_1;
	else
		flags &= ~ZONE_1;
		
	// motion
	if (port1.digiRead() && motion_good())
		flags |= ZONE_2;
	else
		flags &= ~ZONE_2;
		
	// back door
	if (port3.digiRead())
		flags |= ZONE_3;
	else
		flags &= ~ZONE_3;
		
	// garage door	
	if (port4.digiRead())
		flags |= ZONE_4;
	else
		flags &= ~ZONE_4;
		
	// panic button	
	if (port3.digiRead2())
		flags |= ZONE_5;
	else
		flags &= ~ZONE_5;

	if (armed)
		flags |= ARMED;
	else
		flags &= ~ARMED;
		
	if (bypass_zone)
		flags |= BYPASS;
	else
		flags &= ~BYPASS;

#ifdef DEBUG
	if (last_flags != flags) {
		DBG_PRINT("flags=");
		DBG_VAL(flags);
	}
#endif
	key = PC1500RK_transfer(flags);
	
	if (key != last_key && key != NO_KEY_PRESSED) {
		new_key = 1;
		DBG_PRINT("new_key=");
		DBG_VAL(key);
	}
	
	last_key = key;
	
	// beep at beginning of every key press
	if (new_key)
		flags |= BUZZER;
	else
		flags &= ~BUZZER;
	
	// interpret command
	if (new_key) {
		if (key == 'P' || key == 'E' || key == 'F') {
			// emergency command
			emergency = key;
#ifdef SIREN_WHEN_ALARMED
			siren_on();
#endif
			DBG_PRINT("Emergency command");
			return;
		}
		else if (key == '*') {
			// start command
			in_command = 1;
			cmdBuf[0] = key;
			cmd_start = millis();
			flags &= ~READY;
			DBG_PRINT("keypad: start cmd");
			return;
		}
		else if (in_command && cmd_timed_out(cmd_start)) {
			// command timed out
			in_command = 0;
			flags |= READY;
			DBG_PRINT("keypad: cmd timeout");
			return;
		}
		else if (in_command && key <= '9' and key >= '0') {
			// middle of command - store the character
			if (in_command >= CMD_BUFFER_MAX) {
				DBG_PRINT("Max cmd length exceeded");
				in_command=0;
				return;
			}
				
			cmdBuf[in_command] = key;
			in_command++;
			DBG_PRINT("keypad: entering cmd");
			return;
		}
		else if (in_command && key == '#') {
			// finish command
			in_command = 0;
			
			if (cmdBuf[1] == '8' || cmdBuf[1] == '9') {
				// arm system
				if (strncmp((const char*)cmdBuf+2, passcode, PASSCODE_MAX) == 0) {
					if (cmdBuf[1] == '8') {
						// delay arm
						armTimer.set(DELAY_ARM_VALUE);
						armed = 0;
					}
					else {
						// instant arm
						armed = 1;
					}
					alarm = 0;
					flags |= ARMED;
#ifndef MOTION_ALWAYS_ON
					motion_start();
#endif
					siren_off();
					DBG_PRINT("keypad: arm");
				}
#ifdef DEBUG
				else {
					DBG_PRINT("keypad: arm failed");
				}
#endif
			}
			else if (cmdBuf[1] == '7') {
				// disarm system
				if (strncmp((const char*)cmdBuf+2, passcode, PASSCODE_MAX) == 0) {
					armed = 0;
					alarm = 0;
					flags &= ~ARMED;
#ifndef MOTION_ALWAYS_ON				
					motion_stop();
#endif
					siren_off();
					DBG_PRINT("keypad: disarm");
				}
#ifdef DEBUG
				else {
					DBG_PRINT("keypad: disarm failed");
				}
#endif
			}
			else if (cmdBuf[1] == '2') {
				// change passcode
				// TODO: error checks on passcode size, etc.
				DBG_PRINT("keypad: change pwd begin");
				for (i = 0; i < PASSCODE_MAX; i++) {
					EEPROM.write(i, cmdBuf[i+2]);
					passcode[i] = cmdBuf[i+2];
				}
				DBG_PRINT("keypad: change pwd end");
			}
			else if (cmdBuf[1] == '1') {
				// bypass zone
				if (cmdBuf[2] == '0') {
					bypass_zone = 0;
					flags &= ~BYPASS;
					DBG_PRINT("keypad: bypass zone cleared");
				}
				else {
					bypass_zone = cmdBuf[2];
					flags |= BYPASS;
					DBG_PRINT("keypad: bypass zone set");
				}
				
			}
			
			flags |= READY;
			
			return;
		}
	}
	
	return;
}

void loop() {
  static unsigned char last_digital = 0;
  static unsigned char current_digital = 0;
  static int last_analog = 0;
  static int current_analog = 0;
  static char alarm_sent = 0;
  static char last_armed = 0;

  if (rf12_recvDone() && rf12_crc == 0) {
    // if we received something see if it's the right size 
	//   and enable/disable motion sensor or siren
	switch (rf12_data[0]) {
		case 'm':
		  // motion sensor control
		  if (rf12_len != 2)
			break;
		  if (rf12_data[1] == '1') {
			motion_start();
			DBG_PRINT("rfrx: motion start");
		  }
		  else {
			motion_stop();
			DBG_PRINT("rfrx: motion stop");
		  }
		  break;
		case 's':
		  // siren control
		  if (rf12_len != 2)
			break;
		  if (rf12_data[1] == '1') {
			siren_on();
			DBG_PRINT("rfrx: siren on");
		  }
		  else {
			siren_off();
			DBG_PRINT("rfrx: siren off");
		  }
		  break;
		case 'u':
		  // sensor update requested
		  if (rf12_len != 1)
			break;
		  sensor_update = 1;
		  DBG_PRINT("rfrx: sensor update");
		  break;
		case 'a':
		  // arm the system
		  if (rf12_len != 1)
			break;
		  armed = 1;
		  alarm = 0;
		  siren_off();
		  DBG_PRINT("rfrx: arm");
		  break;
		case 'd':
		  // disarm the system
		  if (rf12_len != 1)
			break;
		  armed = 0;
		  alarm = 0;
		  siren_off();
		  DBG_PRINT("rfrx: disarm");
		  break;
		default:
		  // bad command; do nothing
		  DBG_PRINT("rfrx: bad msg");
		  break;
	}
	
    // check to see if an ACK is requested from sender
	// TODO: When an ACK is requested, ACK gets sent but not the subsequent packet
    if (RF12_WANTS_ACK) {
	  rf12_sendStart(RF12_ACK_REPLY, 0, 0);
	  DBG_PRINT("rfrx: send ack");
	}
  }

  if (sendTimer.poll(SENSOR_POLL_RATE)) {
    // output sensor readings
	
    current_digital = 0 | port1.digiRead()
						| (port1.digiRead2() << 1)
						| (port2.digiRead() << 2)
						| (port2.digiRead2() << 3)
						| (port3.digiRead() << 4)
						| (port3.digiRead2() << 5)
						| (port4.digiRead() << 6); 
	
	current_analog = port4.anaRead();
	
	// only send if the values have changed significantly
	if (current_digital != last_digital) {
	  sensor_update = 1;
	  DBG_PRINT("sensors: digital change");
	  DBG_VAL(current_digital);
	}
	else if (last_analog >= SEND_THRESHOLD 
			 && last_analog <= WORD_MAX - SEND_THRESHOLD
			 && (current_analog < last_analog - SEND_THRESHOLD
			     || current_analog > last_analog + SEND_THRESHOLD)) {
	  sensor_update = 1;
	  DBG_PRINT("sensors: analog change");
	  DBG_VAL(current_analog);
	}
  }

  // communicate with keypad
  if (keypadTimer.poll(KEYPAD_POLL_RATE))
	keypad_statemachine();
  
  // see if delay arm is active
  if (armTimer.poll())
	armed = 1;
	
  // turn off siren automatically after the timeout period
  if (siren_timeout()) {
    siren_off();
  }
  
  if (armed && !alarm) {
	if ((bypass_zone != '1' && port2.digiRead()) /* front door */
		|| (bypass_zone != '2' && motion_good() && port1.digiRead()) /* motion */
		|| (bypass_zone != '3' && port3.digiRead()) /* back door */
		|| (bypass_zone != '4' && port4.digiRead()) /* garage door */
		|| (bypass_zone != '5' && port3.digiRead2()) /* panic button */ ) {
		alarm = 1;
		alarm_sent = 0;
#ifdef SIREN_WHEN_ALARM
		siren_on();
#endif
		DBG_PRINT("ALARM!!!");
	}
  }
  
  // send sensor update
  if (sensor_update && rf12_canSend()) {
	outData[0] = MSG_SENSOR;
	outData[1] = current_digital;
	outData[2] = (unsigned char)(current_analog & 0xff);
	outData[3] = (unsigned char)((current_analog >> 8) & 0xff);
	
	last_digital = current_digital;
	last_analog = current_analog;
	
    rf12_sendStart(MASTER_NODEID, outData, 4);
    sensor_update = 0;
	DBG_PRINT("rftx: sensor update");
  }
  
  // send emergency message
  if (emergency && rf12_canSend()) {
	outData[0] = MSG_EMERGENCY;
	outData[1] = emergency;
	rf12_sendStart(MASTER_NODEID, outData, 2);
	emergency = 0;
	DBG_PRINT("rftx: emergency");
  }
  
  // send alarm triggered message
  if (alarm && !alarm_sent && rf12_canSend()) {
	outData[0] = MSG_ALARM;
	rf12_sendStart(MASTER_NODEID, outData, 1);
	alarm_sent = 1;
	DBG_PRINT("rftx: alarm");
  }
  
  // send arm/disarm message
  if (last_armed != armed && rf12_canSend()) {
	outData[0] = MSG_ARM;
	outData[1] = armed;
	rf12_sendStart(MASTER_NODEID, outData, 2);
	last_armed = armed;
	DBG_PRINT("rftx: arm/disarm");
  }
}
