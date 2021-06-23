/*
    This project is a rewrite of the Mechaduino firmware, removing
    test code and replacing the serial commands with a basic GCode 
    parser. 

    Please check out the original Mechaduino project for more details!
*/

#include "Parameters.h"
#include "Startup.h"
#include "analogFastWrite.h"

void setup()
{
    SerialUSB.begin(115200);
    setupSPI();
    setupPins();
    setupTCInterrupts();
}