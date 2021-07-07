
/*
  -------------------------------------------------------------
  Mechaduino 0.1 & 0.2 Firmware  v0.1.5
  SAM21D18 (Arduino Zero compatible), AS5047 encoder, A4954 driver

  All Mechaduino related materials are released under the
  Creative Commons Attribution Share-Alike 4.0 License
  https://creativecommons.org/licenses/by-sa/4.0/

  Many thanks to all contributors!
  --------------------------------------------------------------
  
  Controlled via a SerialUSB terminal at 115200 baud.

  Implemented serial commands are:

*/

#include "Utils.h"
#include "Parameters.h"
#include "State.h"
#include "analogFastWrite.h"

/////////////////SETUP////////////////
// Initialize variables for periodically reporting torque and position
const long timeDelay = 50; // Reporting period, in milliseconds
long currTime;
long prevTime;

void setup()        // This code runs once at startup
{                         
   
  digitalWrite(ledPin,HIGH);        // turn LED on 
  setupPins();                      // configure pins
  setupTCInterrupts();              // configure controller interrupt

  SerialUSB.begin(115200);          
  delay(3000);                      // This delay seems to make it easier to establish a connection when the Mechaduino is configured to start in closed loop mode.  
  serialMenu();                     // Prints menu to serial monitor
  setupSPI();                       // Sets up SPI for communicating with encoder
  digitalWrite(ledPin,LOW);         // turn LED off 
  
  // spot check some of the lookup table to decide if it has been filled in
  if (lookup[0] == 0 && lookup[128] == 0 && lookup[1024] == 0)
    SerialUSB.println("WARNING: Lookup table is empty! Run calibration");

  currTime = millis();              // Initialize variables
  prevTime = currTime;

  // Print out formatting for periodic outputs in CSV format
  SerialUSB.print("\"time\", \"position\", \"effort\" \n\r");
}
  
////////////////LOOP/////////////////
void loop()                 // main loop
{

  serialCheck();              //must have this execute in loop for serial commands to function

  // Every timeDelay milliseconds, report on the status
//  currTime = millis();
//  if(prevTime + timeDelay < currTime){
//    prevTime = currTime;
//    // Send out the current time, the position, and the "effort"
//    SerialUSB.print(String(currTime) + ", " + String(yw) + ", " + String(u) + "\n\r");
//  }
}
