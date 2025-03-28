//Contains the TC5 Handler declaration


#ifndef __CONTROLLER_H__
#define  __CONTROLLER_H__

#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);
// For rolling average filter
#define FILTER_LEN    32
extern volatile float u_past[FILTER_LEN];
extern volatile unsigned int   controller_flag;
#define FILTER_PERIOD_US 20000 // Time period between adding to the filter

// For movement control
#define MOVE_CTRL_HZ     500.0 // Frequency of movement control updates

// Flags for the controller
#define MAX_EFFORT_ERR 0    // Bit number for indicating effort too high
#define MAX_SLOPE_ERR  1    // Bit indicates a sharp spike in effort has been detected
#define BUSY           2    // Bit number for indicating movement in progress
#define MISC_NOTIF     3    // Bit number for general purpose flag
#define NO_FLAGS       0
// Flags for indicating behavior (might as well do them here and save 4 bytes)
#define UNITS_MM            4
#define POS_ABSOLUTE        5
// Flag for indicating whether we are debugging
#define DEBUG_MODE          6
// Flags for the loop to set
#define COMMAND_SHIFT   7
#define COMMAND_MASK    (0b111 << COMMAND_SHIFT)
#define NULL_COMMAND    0b000 // Nothing
#define STOP_COMMAND    0b111 // Halt
#define MOVE_COMMAND    0b001 // Rapid move
#define LINEAR_COMMAND  0b010 // Linear move
#define DWELL_COMMAND   0b011 // Dwell

// Used to determine whether we are very close to our target
#define SMALL_DIST_LIMIT    0.5 // measured in degrees


void TC5_Handler();
void TC4_Handler();


#endif