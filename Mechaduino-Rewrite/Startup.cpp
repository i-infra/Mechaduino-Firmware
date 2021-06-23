// Implements the functions necessary for setup

#include "Parameters.h"      // Includes macros, constants, and global variables
#include "analogFastWrite.h" // Implements a higher frequency analogWrite
#include <SPI.h>             // SPI library for communcating with encoder

// Initialize pins
void setupPins() {
	// Set output pins as output
	// 1-pin DACs set voltage references
		// Pins VREF_1 and VREF_2 connect to a RC LPF, creating a voltage that
		// is closer to the expected DC value.
		// This reference voltage connects to the PWM motor controller
		// and is used to set the maximum current output
    pinMode(VREF_2, OUTPUT);
    pinMode(VREF_1, OUTPUT);
    // 4 pins for the PWM motor driver
    pinMode(IN_4, OUTPUT);
    pinMode(IN_3, OUTPUT);
    pinMode(IN_2, OUTPUT);
    pinMode(IN_1, OUTPUT);


    // Set the current limit. 
    // I_max = V_max/R_sense = V_REF/(10*R_sense)
    // Assuming a perfect LPF smoothing out the PWM DAC and a 3.3V power source,
    // V_REF here is set to be some value so the motor controller knows to wake.
    analogFastWrite(VREF_2, 0.3 * uMAX);
    analogFastWrite(VREF_1, 0.3 * uMAX);

    // Brake the motor initially
    IN_4_LOW();
    IN_3_LOW();
    IN_2_LOW();
    IN_1_LOW();

    return;
}

// Initialize SPI communications with the magnetic encoder
void setupSPI() {
	// Set chip select pin as an output
    pinMode(chipSelectPin, OUTPUT); 

    // Configure settings
    CHIPSELECT_LOW();
    // Set max speed, bit order, and mode
    SPISettings settingsA(ENCODER_MAXSPEED, ENCODER_BITORDER, ENCODER_SPIMODE);
    SPI.begin();
    delay(ENCODER_STARTUP);
    SPI.beginTransaction(settingsA);
    // Done with configuring settings
    CHIPSELECT_HIGH();

    return;
}

// Initialize timed interrupts. This is copied directly from the Mechaduino
// firmware with a few added comments
void setupTCInterrupts() {  // configure the controller interrupt

  // Enable GCLK for TC4 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable Time Counter x
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16; // Set Timer mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as Normal Frq
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;// Set prescaler: divide by
  WAIT_TC16_REGS_SYNC(TC5)                          // one for max speed

  TC5->COUNT16.CC[0].reg = TRIGGER_PER;
  WAIT_TC16_REGS_SYNC(TC5) // Set the trigger period, clock freq / trigger freq

  TC5->COUNT16.INTENSET.reg = 0;             // disable all interrupts
  TC5->COUNT16.INTENSET.bit.OVF = 1;         // enable overflow
  TC5->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0

  NVIC_SetPriority(TC5_IRQn, 1);              //Set interrupt priority

  // Enable InterruptVector
  NVIC_EnableIRQ(TC5_IRQn);
}