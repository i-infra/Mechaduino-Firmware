//Contains the TC5 Handler declaration


#ifndef __CONTROLLER_H__
#define  __CONTROLLER_H__

#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);
// For rolling average filter
#define FILTER_LEN    32
extern volatile float u_past[FILTER_LEN];
extern volatile int   controller_flag;
#define FILTER_PERIOD_US 20000 // Time period between adding to the filter

// Flags for the controller
#define MAX_EFFORT_ERR 1<<0
#define NO_FLAGS       0

void TC5_Handler();


#endif