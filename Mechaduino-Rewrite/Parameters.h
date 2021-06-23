// Contains declarations for variables and constants used by firmware.

#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

// Macros for pins. These are referencing the Arduino core pins, not the 
// physical pins on the SAMD21.
#define IN_4   6   // These pins are for controlling the stepper motor direction
#define IN_3   5   
#define IN_2   7
#define IN_1   8
#define VREF_2 4  // These control the amount of current going to the steppers.
#define VREF_1 9

#define chipSelectPin A2 // Used for controlling the encoder


// Macros for writing to the data data registers directly.
// This is faster than using digitalWrite, but if the pins change, these also
// must be updated.
#define IN_1_HIGH() (REG_PORT_OUTSET0 = PORT_PA06)
#define IN_1_LOW() (REG_PORT_OUTCLR0 = PORT_PA06)
#define IN_2_HIGH() (REG_PORT_OUTSET0 = PORT_PA21)
#define IN_2_LOW() (REG_PORT_OUTCLR0 = PORT_PA21)
#define IN_3_HIGH() (REG_PORT_OUTSET0 = PORT_PA15)
#define IN_3_LOW() (REG_PORT_OUTCLR0 = PORT_PA15)
#define IN_4_HIGH() (REG_PORT_OUTSET0 = PORT_PA20)
#define IN_4_LOW() (REG_PORT_OUTCLR0 = PORT_PA20)
#define CHIPSELECT_HIGH() (REG_PORT_OUTSET1 = PORT_PB09)
#define CHIPSELECT_LOW() (REG_PORT_OUTCLR1 = PORT_PB09)

// Macros for proper SPI communications with the encoder. See the datasheet for
// the encoder for how these should be set.
#define ENCODER_MAXSPEED   10000000
#define ENCODER_BITORDER   MSBFIRST
#define ENCODER_SPIMODE    SPI_MODE1

#define ENCODER_STARTUP    500       // Milliseconds it takes to start SPI

// Macros and variable declarations for timing
#define CLK_SPEED          (float)48000000 // Speed of the SAMD21, Hz
#define TRIGGER_FREQ       (float)6500     // Freqency of interrupts, Hz
extern const float Fs;
#define TRIGGER_PER        (int)(round(CLK_SPEED/Fs))


#endif