//Contains TC5 Controller definition
//The main control loop is executed by the TC5 timer interrupt:

#include <SPI.h>

#include "State.h"
#include "Utils.h"
#include "Parameters.h"
#include "Controller.h"

// Initialize controller flag and rolling average
volatile unsigned int controller_flag = NO_FLAGS;
volatile float u_roll = 0;
volatile float u_roll_1 = 0;
volatile float u_past[FILTER_LEN];
volatile unsigned long past_filter_time = 0;
volatile unsigned long past_move_time = 0;

void TC5_Handler() {// gets called with FPID frequency, defined in Parameters
  if (TC5->COUNT16.INTFLAG.bit.OVF == 1) {        // A counter overflow caused the interrupt
    y = lookup[readEncoder()];                    //read encoder and lookup corrected angle in calibration lookup table

    if ((y - y_1) < -180.0) wrap_count += 1;      //Check if we've rotated more than a full revolution (have we "wrapped" around from 359 degrees to 0 or ffrom 0 to 359?)
    else if ((y - y_1) > 180.0) wrap_count -= 1;

    yw = (y + (360.0 * wrap_count));              //yw is the wrapped angle (can exceed one revolution)

    switch (mode) {
      case 'x':         // position control
        e = (r - yw);

        ITerm += (pKi * e);                             //Integral wind up limit
        if (ITerm > 150.0) ITerm = 150.0;
        else if (ITerm < -150.0) ITerm = -150.0;

        DTerm = pLPFa * DTerm -  pLPFb * pKd * (yw - yw_1);

        u = (pKp * e) + ITerm + DTerm;


        break;

      case 'v':         // velocity controlr
        v = vLPFa * v +  vLPFb * (yw - yw_1); //filtered velocity called "DTerm" because it is similar to derivative action in position loop

        e = (r - v);   //error in degrees per rpm (sample frequency in Hz * (60 seconds/min) / (360 degrees/rev) )

        ITerm += (vKi * e);                 //Integral wind up limit
        if (ITerm > 200) ITerm = 200;
        else if (ITerm < -200) ITerm = -200;

        u = ((vKp * e) + ITerm - (vKd * (e - e_1)));

        //SerialUSB.println(e);
        break;

      default:
        u = 0;
        break;
    }

    y_1 = y;  //copy current value of y to previous value (y_1) for next control cycle before PA angle added


    if (u > 0)          //Depending on direction we want to apply torque, add or subtract a phase angle of PA for max effective torque.  PA should be equal to one full step angle: if the excitation angle is the same as the current position, we would not move!
    { //You can experiment with "Phase Advance" by increasing PA when operating at high speeds
      y += PA;          //update phase excitation angle
      if (u > uMAX)     // limit control effort
        u = uMAX;       //saturation limits max current command
    }
    else
    {
      y -= PA;          //update phase excitation angle
      if (u < -uMAX)    // limit control effort
        u = -uMAX;      //saturation limits max current command
    }

    U = abs(u);       //

    if (abs(e) < 0.1) ledPin_HIGH();    // turn on LED if error is less than 0.1
    else ledPin_LOW();                  //digitalWrite(ledPin, LOW);

    output(-y, round(U));    // update phase currents
  }

  //copy current values to previous values for next control cycle
  //e_2 = e_1;    //these past values can be useful for more complex controllers/filters.  Uncomment as necessary
  e_1 = e;
  //u_2 = u_1;
  u_1 = u;
  yw_1 = yw;
  
  // Stop moving if effort is exceeded
  if(U > EFFORT_MAX && mode == 'v'){
    r = 0;
    // Set the MAX_EFFORT_ERR flag
    controller_flag |= 1<<MAX_EFFORT_ERR;
  }

  // Shift in new value and take the average to get the filtered effort
  // Do the following with period FILTER_PERIOD_US
  unsigned long current_time = micros();  
  if((current_time - FILTER_PERIOD_US) > past_filter_time){
    u_roll_1 = u_roll;
    u_roll = 0;
    past_filter_time = current_time;
    
    for(int i = FILTER_LEN-1; i >= 1; i--){
      // Shift in a new value and do avg
      u_past[i] = u_past[i-1];
      u_roll += u_past[i];
    }
    u_past[0] = u;
    u_roll += u;
    u_roll = u_roll / FILTER_LEN;
  }

  // This is the bit of code that manages movement so the
  // main loop can do other stuff.
  // We don't need to set the speed or check the time during
  // every cycle; set the period with MOVE_CTRL_US

  float time, velocity; //, sign;
  // Consider the cases: 
  unsigned int command = (controller_flag & COMMAND_MASK)>>COMMAND_SHIFT;
  if((current_time - MOVE_CTRL_US) > past_move_time){
    past_move_time = current_time;
    switch(command){
      case STOP_COMMAND:
        break;
      case MOVE_COMMAND:
        // We are busy until we are at the target destination.
        controller_flag &= ~((abs(yw-r) < SMALL_DIST_LIMIT)<<BUSY);
        break;
      case LINEAR_COMMAND:
        // again, we are busy until we reach our target
        controller_flag &= ~((abs(yw-target) < SMALL_DIST_LIMIT)<<BUSY);
        // if we haven't reached our target yet, get moving
        if (controller_flag & 1<<BUSY){
          // if accel = 0, go at const speed.
          if(data6 == 0){
            r = bound_vel(data3);
            time = 0;
            velocity = r;
          }
          // otherwise, speed is f(position)
          else{
            time = abs(data1 * (data2 + sqrt(data3*data3 - data4*(data5-yw))));
            velocity = data3 + data6 * time;
            r = bound_vel(velocity);
          }
        }
        // if we did reach the target, stop moving
        else {
          mode = 'x';
          r = target;
        }
        // time to do floating point math to set the velocity
        
        // overshoot correction: if r*MOVE_CTRL_US makes us overshoot, just go to the position
        // sign = (r>0) - (r<0);
        // if((sign * (r * MOVE_CTRL_US *360 /(1000000*60)) > sign * (target-yw)) || !(controller_flag&(1<<BUSY))){
        //   mode = 'x';
        //   r = target;
        // }
        break;
      case DWELL_COMMAND:
        // millis()-data1 is milliseconds elaped since the dwell was called
        // Thus, the busy bit is only cleared target time has elapsed.
        controller_flag &= ~(((millis()-data1) > target)<<BUSY);
        break;

    }
  }

  
  TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
}
