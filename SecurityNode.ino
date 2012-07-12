// Security Node
// This sketch is for a JeeNode attached to a security system
// Copyright (c) 2012 Ken Sharp
// License: http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>

#define POLL_RATE 2000  // in ms
#define MASTER_NODEID 1

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
  //rf12_recvDone();
  if (rf12_recvDone() && rf12_crc == 0) {
    // if we received something see if it's the right size 
	//   and enable/disable motion sensor or siren
    if (rf12_len == 2) {
	  switch (rf12_data[0]) {
	    case 'm':
	      if (rf12_data[1] == '1')
		    port1.digiWrite2(1);
	      else
		    port1.digiWrite2(0);
		  break;
		case 's':
	      if (rf12_data[1] == '1')
		    port2.digiWrite2(1);
	      else
		    port2.digiWrite2(0);
		  break;
		default:
		  // bad command; do nothing
		  break;
	  }
	}
	
    // check to see if an ACK is requested from sender
    if (RF12_WANTS_ACK)
	  rf12_sendStart(RF12_ACK_REPLY, 0, 0);
  }

  if (sendTimer.poll(2000)) {
    // output sensor readings
    outData[0] = 0 | port1.digiRead()
				   | (port1.digiRead2() << 1)
				   | (port2.digiRead() << 2)
				   | (port2.digiRead2() << 3)
				   | (port3.digiRead() << 4)
				   | (port3.digiRead2() << 5)
				   | (port4.digiRead() << 6); 
				   
    outData[1] = port4.anaRead();
	
    pending = 1;
  }

  if (pending && rf12_canSend()) {
    rf12_sendStart(MASTER_NODEID, outData, sizeof outData);
    pending = 0;
  }
}
