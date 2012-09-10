// Security Node
// This sketch is for a JeeNode attached to a security system
// Copyright (c) 2012 Ken Sharp
// License: http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define MASTER_NODEID 1
#define POLL_RATE 500  // in ms
#define SEND_THRESHOLD 2 // in ADC counts

// according to http://arduino.cc/en/Reference/Word
#define WORD_MAX 65535

byte pending;
word outData[2];
MilliTimer sendTimer;
Port port1(1);
Port port2(2);
Port port3(3);
Port port4(4);

void setup() {

  rf12_initialize(2, RF12_915MHZ, 212);

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
}

void loop() {
  word last_digital = 0;
  word last_analog = 0;

  //rf12_recvDone();
  if (rf12_recvDone() && rf12_crc == 0) {
    // if we received something see if it's the right size 
	//   and enable/disable motion sensor or siren
    if (rf12_len == 2) {
	  switch (rf12_data[0]) {
	    case 'm':
		  // motion sensor control
	      if (rf12_data[1] == '1')
		    port1.digiWrite2(1);
	      else
		    port1.digiWrite2(0);
		  break;
		case 's':
		  // siren control
	      if (rf12_data[1] == '1')
		    port2.digiWrite2(1);
	      else
		    port2.digiWrite2(0);
		  break;
		case 'u':
		  // sensor update requested
		  pending = 1;
		default:
		  // bad command; do nothing
		  break;
	  }
	}
	
    // check to see if an ACK is requested from sender
    if (RF12_WANTS_ACK)
	  rf12_sendStart(RF12_ACK_REPLY, 0, 0);
  }

  if (sendTimer.poll(POLL_RATE)) {
    // output sensor readings
    outData[0] = 0 | port1.digiRead()
				   | (port1.digiRead2() << 1)
				   | (port2.digiRead() << 2)
				   | (port2.digiRead2() << 3)
				   | (port3.digiRead() << 4)
				   | (port3.digiRead2() << 5)
				   | (port4.digiRead() << 6); 
				   
    outData[1] = port4.anaRead();
	
	// only send if the values have changed significantly
	if (outData[0] != last_digital)
	  pending = 1;
	else if (last_analog >= SEND_THRESHOLD 
			 && last_analog <= WORD_MAX - SEND_THRESHOLD
			 && (outData[1] < last_analog - SEND_THRESHOLD
			     || outData[1] > last_analog + SEND_THRESHOLD))
	  pending = 1;
  }

  if (pending && rf12_canSend()) {
	last_digital = outData[0];
	last_analog = outData[1];
	
    rf12_sendStart(MASTER_NODEID, outData, sizeof outData);
    pending = 0;
  }
}
