// RFTransmitter - Version: Latest 
#include <RFTransmitter.h>
  // https://andreasrohner.at/posts/Electronics/New-Arduino-library-for-433-Mhz-AM-Radio-Modules/

// TinySnore - Version: Latest 
//#include <tinysnore.h>
  // Send ATtiny to sleep in one line

/*
Small module to send a signal that power is on.

The module is to be controlled by a switch activated power outlet. 
     When power is available on the outlet, the switch must be 'ON', therefore send signal to controller unit
The controller unit, a distance from the outlet being controlled, can then switch in a local power source/outlet to control something.

*/

// RF Rx Module (433 MHz ASK Module)
const byte RadioOut_PIN = PB1; 
const byte RFNodeID = 0;                    // Set this unit to Node # expeected by Rx unit  - Rx unit uses this to confirm valid message
char *RFData = "I'm Alive";           // Only valid message expected by Rx
const byte MaxRFDataSize = 10;
RFTransmitter RFTx(RadioOut_PIN,RFNodeID);               // Set up RF Tx library on RadioOut Pin, set NodeID


void setup() {
  // RF Rx Module (433 MHz ASK Module)
  pinMode(RadioOut_PIN, OUTPUT); 
//  RFTx.begin();                               //Start the Library   - not in the TX library
}



void loop() {
    RFTx.send((byte *)RFData, strlen(RFData) + 1);
    delay(250);                                 // Resend message every 250 ms
}
