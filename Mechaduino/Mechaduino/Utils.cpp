//Contains the definitions of the functions used by the firmware.


#include <SPI.h>
#include <Wire.h>
#include <FlashStorage.h>

#include "Parameters.h"
#include "Controller.h"
#include "Utils.h"
#include "State.h"
#include "analogFastWrite.h"

void setupPins() {

  pinMode(VREF_2, OUTPUT);
  pinMode(VREF_1, OUTPUT);
  pinMode(IN_4, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_1, OUTPUT);

  pinMode(chipSelectPin, OUTPUT); // CSn -- has to toggle high and low to signal chip to start data transfer


  pinMode(ledPin, OUTPUT); //

  // pinMode(clockPin, OUTPUT); // SCL    for I2C
  // pinMode(inputPin, INPUT); // SDA


  analogFastWrite(VREF_2, 0.33 * uMAX);
  analogFastWrite(VREF_1, 0.33 * uMAX);

  IN_4_HIGH();   //  digitalWrite(IN_4, HIGH);
  IN_3_HIGH();    //  digitalWrite(IN_3, LOW);
  IN_2_HIGH();   //  digitalWrite(IN_2, HIGH);
  IN_1_HIGH();    //  digitalWrite(IN_1, LOW);
}

void setupSPI() {

  SPISettings settingsA(10000000, MSBFIRST, SPI_MODE1);             ///400000, MSBFIRST, SPI_MODE1);

  SPI.begin();    //AS5047D SPI uses mode=1 (CPOL=0, CPHA=1)
  SerialUSB.println("Beginning SPI communication with AS5047 encoder...");
  delay(1000);
  SPI.beginTransaction(settingsA);

}

void configureStepDir() {
  pinMode(step_pin, INPUT);
  pinMode(dir_pin, INPUT);
  attachInterrupt(step_pin, stepInterrupt, RISING);
  attachInterrupt(dir_pin, dirInterrupt, CHANGE);
}

void configureEnablePin() {
  pinMode(enable_pin, INPUT);
  attachInterrupt(enable_pin, enableInterrupt, CHANGE);
}


void stepInterrupt() {
  if (dir) r += stepangle;
  else r -= stepangle;
}

void dirInterrupt() {
  if (REG_PORT_IN0 & PORT_PA11) dir = false; // check if dir_pin is HIGH
  else dir = true;
}

void enableInterrupt() {            //enable pin interrupt handler
  if (REG_PORT_IN0 & PORT_PA14){   // check if enable_pin is HIGH
    disableTCInterrupts();
    analogFastWrite(VREF_2, 0);  //set phase currents to zero
    analogFastWrite(VREF_1, 0);
    }
  else{
    enableTCInterrupts();    
    }
}

void output(float theta, int effort) {
   int angle_1;
   int angle_2;
   int v_coil_A;
   int v_coil_B;

   int sin_coil_A;
   int sin_coil_B;
   int phase_multiplier = 10 * spr / 4;

  //REG_PORT_OUTCLR0 = PORT_PA09; for debugging/timing

  angle_1 = mod((phase_multiplier * theta) , 3600);   //
  angle_2 = mod((phase_multiplier * theta)+900 , 3600);
  
  sin_coil_A  = sin_1[angle_1];

  sin_coil_B = sin_1[angle_2];

  v_coil_A = ((effort * sin_coil_A) / 1024);
  v_coil_B = ((effort * sin_coil_B) / 1024);

/*    // For debugging phase voltages:
     SerialUSB.print(v_coil_A);
     SerialUSB.print(",");
     SerialUSB.println(v_coil_B);
*/
  analogFastWrite(VREF_1, abs(v_coil_A));
  analogFastWrite(VREF_2, abs(v_coil_B));

  if (v_coil_A >= 0)  {
    IN_2_HIGH();  //REG_PORT_OUTSET0 = PORT_PA21;     //write IN_2 HIGH
    IN_1_LOW();   //REG_PORT_OUTCLR0 = PORT_PA06;     //write IN_1 LOW
  }
  else  {
    IN_2_LOW();   //REG_PORT_OUTCLR0 = PORT_PA21;     //write IN_2 LOW
    IN_1_HIGH();  //REG_PORT_OUTSET0 = PORT_PA06;     //write IN_1 HIGH
  }

  if (v_coil_B >= 0)  {
    IN_4_HIGH();  //REG_PORT_OUTSET0 = PORT_PA20;     //write IN_4 HIGH
    IN_3_LOW();   //REG_PORT_OUTCLR0 = PORT_PA15;     //write IN_3 LOW
  }
  else  {
    IN_4_LOW();     //REG_PORT_OUTCLR0 = PORT_PA20;     //write IN_4 LOW
    IN_3_HIGH();    //REG_PORT_OUTSET0 = PORT_PA15;     //write IN_3 HIGH
  }

}

static FlashClass flash;
static const unsigned page_size = 256; // actual size is 64?
static unsigned page_count;
static const unsigned floats_per_page = page_size / sizeof(float);
static float page[floats_per_page];
static const void * page_ptr;

static void write_page()
{
  flash.erase((const void*) page_ptr, sizeof(page));
  flash.write((const void*) page_ptr, (const void *) page, sizeof(page));
}

static void store_lookup(float lookupAngle)
{
  page[page_count++] = lookupAngle;
  if(page_count != floats_per_page)
    return;

  // we've filled an entire page, write it to the flash
  write_page();

  // reset our counters and increment our flash page
  page_ptr += sizeof(page);
  page_count = 0;
  memset(page, 0, sizeof(page));
}


int calibrate() {   /// this is the calibration routine

  int encoderReading = 0;
  int currentencoderReading = 0;
  int lastencoderReading = 0;
  int avg = 16;

  int iStart = 0;     //encoder zero position index
  int jStart = 0;
  
  int fullStepReadings[spr];
    
  int fullStep = 0;
  int ticks = 0;
  float lookupAngle = 0.0;
  SerialUSB.println("Beginning calibration routine...");
  // Take a couple steps to make sure it's working right
  oneStep();
  delay(SETTLE_TIME);
  oneStep();
  delay(READ_TIME);
  for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
    currentencoderReading = mod(readEncoder(),cpr);
    encoderReading += currentencoderReading;
    delay(READ_TIME);
  }
  encoderReading/=avg;
  dir = CW;  // Take a step clockwise, wait for vibrations to settle
  oneStep();
  delay(SETTLE_TIME);
  for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
    currentencoderReading = mod(readEncoder(),cpr);
    lastencoderReading += currentencoderReading;
    delay(READ_TIME);
  }
  lastencoderReading/=avg;
  // Take the difference
  currentencoderReading = (lastencoderReading - encoderReading);
  // Wired backwards if:
  // 1) we see a rollover from low to high (should be high to low)
  // 2) we see a small step lower (should be higher)
  SerialUSB.println(String(currentencoderReading));
  if(currentencoderReading > cpr/2 || currentencoderReading<0){
    SerialUSB.println("Try again. If problem persists, swap wiring");
    return CALIBRATION_FAIL;
  }
  while (stepNumber != 0) {       //go to step zero
    if (stepNumber > 0) {
      dir = CW;
    }
    else
    {
      dir = CCW;
    }
    oneStep();
    delay(QUICK_SETTLE/2);
  }
  
  dir = CW;

  for (int x = 0; x < spr; x++) {     //step through all full step positions, recording their encoder readings

    encoderReading = 0;               // init. as 0 for averages
    delay(QUICK_SETTLE);                        //moving too fast may not give accurate readings.  Motor needs time to settle after each step.
    lastencoderReading = readEncoder();
        
    for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
      currentencoderReading = readEncoder();

      // If we are on the edge of wrapping around, add
      // or subtract as needed to keep the value correct
      if ((currentencoderReading-lastencoderReading)<(-(cpr/2))){
        currentencoderReading += cpr;
      }
 
      encoderReading += currentencoderReading;
      delay(READ_TIME);
    }
    // Take the average
    encoderReading = encoderReading / avg;
    // Put it back in range of the 14 bit value
    if (encoderReading>=cpr){
      encoderReading-= cpr;
    }
    else if (encoderReading<0){
      encoderReading+= cpr;
    }

    fullStepReadings[x] = encoderReading;
    
    // go to next step
    oneStep();
  }
  // Once we know everything, we can analyze the data.
  findijStart(fullStepReadings, &iStart, &jStart);

  // The code below generates the lookup table by intepolating between
  // full steps and mapping each encoder count to a calibrated angle
  // The lookup table is too big to store in volatile memory,
  // so we must generate and store it into the flash on the fly

  // begin the write to the calibration table
  page_count = 0;
  page_ptr = (const uint8_t*) lookup;
  // Start counting at iStart
  for (int i = iStart; i < (iStart + spr + 1); i++) {
    ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];

    if (ticks < -cpr/2) {           //check if current interval wraps over encoder's zero positon
      ticks += cpr;
    }
    if (i == iStart) { //this is an edge case
      // starting at 0, go through the ticks and assign an angle
      // given that 1 tick = 1 aps
      // For this case, we only care about the vals between jStart and ticks
      for (int j = jStart; j < (ticks); j++) {
	      store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
      }
    }
    else if (i == (iStart + spr)) { //this is an edge case
      // this time, we are ending at 0, making sure not to double-count
      // the ones covered in the previous case
      for (int j = 0; j < jStart; j++) {
	     store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
      }
    }
    else {                        //this is the general case
      for (int j = 0; j < ticks; j++) {
	      store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
      }
    }
  }

  // Store unwritten page
  if (page_count != 0)
	write_page();
  SerialUSB.println("The calibration table has been written to non-volatile Flash memory!");
  return CALIBRATION_SUCCESS;
}

void findijStart(int readings[], int* istart, int* jstart){
  int ticks;
  int stepNo;
  // We know readings[] will always be spr long
  for(int i =0; i<spr; i++){
    // Take the difference of two consecutive items (wrapping around)
    ticks = readings[mod((i + 1), spr)] - readings[mod((i), spr)];
    // if a step causes wrapping around, add as needed
    if (ticks < -cpr/2) {
      ticks += cpr;
    }
    // We now have all positive ticks (enforced by calibration routine)
    for (int j = 0; j < ticks; j++){
      // Interpolate between start and end val of the tick to find
      // when the encoder would read 0
      stepNo = (mod(readings[i] + j, cpr));
      if(stepNo==0){
        // Record the step number and 
        *istart = i;
        *jstart = j;
        return;
      }
    }
  }
  return;
}


float read_angle()
{
  const int avg = 10;            //average a few readings
  int encoderReading = 0;

  disableTCInterrupts();        //can't use readEncoder while in closed loop

  for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
    encoderReading += readEncoder();
    delay(10);
    }

  //return encoderReading * (360.0 / 16384.0) / avg;
  return lookup[encoderReading / avg];
}


void serialCheck() {        //Monitors serial for commands.  Must be called in routinely in loop for serial interface to work.

  if (SerialUSB.available()>0) {

    char inChar = (char)SerialUSB.peek();
    if(IS_CAPITAL(inChar)){ // Capital letters indicate GCode commands.
      gcode_parse();        // Parse the command and return from serialCheck
      return;
    }

    SerialUSB.read();       // Otherwise, remove the character from the queue
    
    switch (inChar) {
      case 'p':             //print
        print_angle();
        break;

      case 's':             //step
        oneStep();
        print_angle();
        break;

      case 'd':             //dir
        if (dir) {
          dir = false;
        }
        else {
          dir = true;
        }
        break;

      case 'w':                //old command
        calibrate();           //cal routine
        break;
        
      case 'c':
        calibrate();           //cal routine
        break;        

      case 'e':
        readEncoderDiagnostics();   //encoder error?
        break;

      case 'y':
        r = (read_angle()+(360.0 * wrap_count));          // hold the current position
        SerialUSB.print("New setpoint ");
        SerialUSB.println(r, 2);
        enableTCInterrupts();      //enable closed loop
        break;

      case 'n':
        disableTCInterrupts();      //disable closed loop
        analogFastWrite(VREF_2, 0);     //set phase currents to zero
        analogFastWrite(VREF_1, 0);                       
        break;

      case 'r':             //new setpoint
        SerialUSB.println("Enter setpoint:");
        while (SerialUSB.available() == 0)  {}
        r = SerialUSB.parseFloat();
        SerialUSB.println(r);
        break;

      case 'x':
        mode = 'x';           //position loop
        break;

      case 'v':
        mode = 'v';           //velocity loop
        break;

      case 't':
        mode = 't';           //torque loop
        break;

      case 'h':               //hybrid mode
        mode = 'h';
        break;

      case 'q':
        parameterQuery();     // prints copy-able parameters
        break;

      case 'a':             //anticogging
        antiCoggingCal();
        break;

      case 'k':
        parameterEditmain();
        break;
        
      case 'g':
        sineGen();
        break;

      case 'm':
        serialMenu();
        break;


      default:
        break;
    }
  }

}

void gcode_parse(){
  char command[COMMAND_SIZE];
  byte serial_count = 0;
  char inChar;
  byte delays = 0;
  
  while(delays < DELAY_COUNT){
    if(SerialUSB.available() > 0){
      // If we have serial data, read it.
      inChar = SerialUSB.read();
      // If we reach the end of the command, process it and return
      // out of this function; we're done here.
      if(END_OF_LINE(inChar) && serial_count>0){
       process_string(command, serial_count);
        return;
      }
      // Otherwise, keep reading the command in.
      command[serial_count++] = inChar;
    }
    else {
      // If we don't have serial data, give it a few microseconds to come it
      // But if we have to wait too long, exit this loop
      delays++;
      delayMicroseconds(DELAY_TIME);
    }
  }
  // If we run out of chars in the buffer without hitting a newline,
  // that's techincally out of spec but we will parse the string
  // anyways, as long as it's nonempty.
  if(serial_count>0){
      process_string(command, serial_count);
  }
  // Otherwise, we're done here, nothing more to do.
  return;
}

void process_string(char instruction[], int len){
   // Turn the string into some actual commands!
   // Commands To Implement:
   //   G0:      Rapid Move - Uses the trapezoidal movement pattern, max speed
   //   G1:      Linear Move - Move at speed set by speed set command
   //   G20/G21: Set Units to Inches/Millimeters
   //   G28:     Home/Level - Find max and min displacement of motor and home it
   //   G90/91:  Absolute/Relative position
   //   G92:     Set new home
 

  unsigned int code;
  code = search_code('G', instruction, len);
  if (code != NOT_FOUND){
    // If the G code is found, call the G code handling function
    process_g(code, instruction, len);
    // Nothing left to do in this function
    return;
  }
  code = search_code('M', instruction, len);
  if (code != NOT_FOUND){
    // If the G code is found, call the M code handling function
    process_m(code, instruction, len);
    // Nothing left to do in this function
    return;
  }

  // The only way to get here is if the code is neither a g code
  // nor an m code - the code isn't implemented yet so we return
  SerialUSB.println("Code not found!");
  return;
}

//look for the number that appears after the char key and return it
float search_code(char key, char instruction[], int string_size)
{
  // Temp char string for holding the code
  // Codes are always CODE_LEN or fewer characters long
  char temp[CODE_LEN] = "";

  // Search through the string
  for (byte i=0; i<string_size; i++)
  {
    // When the key is found, search through that area
    if (instruction[i] == key)
    {
      i++;      
      int k = 0;
      while (i < string_size && k < CODE_LEN)
      {
        // If the character isn't a number, stop reading it
        if (!IS_NUMBER(instruction[i]))
          break;
        // Otherwise, add it to temp
        temp[k] = instruction[i];
        i++;
        k++;
      }
      // Return the string turned into a float
      if(temp == ""){
        // Return EMPTY if there is no command
        return EMPTY;
      }
      return String(temp).toFloat();
    }
  }
  // Othewise, say it was not found.
  return NOT_FOUND;
}

float interpolate_pos(float target){
  // Convert the target position in millimeters to the target degree rotation
  float result;
  if(controller_flag & 1<<UNITS_MM){
    result = (float)target * (360.0/((float)MM_PER_ROT));
  }
  else{
    result = (float)target * (360.0/((float)IN_PER_ROT));
  }
  return result;
}

float interpolate_vel(float target){
  // Convert the target velocity in millimeters/min to the target rot/min
  float result;
  if(controller_flag & 1<<UNITS_MM){
    result = (float)target /((float)MM_PER_ROT);
  }
  else{
    result = (float)target /((float)IN_PER_ROT);
  }
  return abs(result);
}


float bound_pos(float target){
  // Check if bounds are exceeded
  // These variables are poorly named... 
  if(target > xmin){
    target = xmin;
  }
  else if(target < xmax){
    target = xmax;
  }

  return target;
}

float bound_vel(float speed){
  // Check if speed bounds are exceeded
  // Takes a speed in RPM (not mm/min or in/min)
  // and outputs the bounded feedrate
  float abs_speed = abs(speed);
  if(abs_speed > MAX_SPEED){
    abs_speed = MAX_SPEED;
  }
  else if(abs_speed < MIN_SPEED){
    abs_speed = MIN_SPEED;
  }

  // Return the adjusted velocity (with correct sign)
  return abs_speed * (speed/abs(speed));
}

void linear_move_action(float reading_x, float reading_misc){
  // This linear move first assumes the target point is possible to reach
  // and all velocites are possible. It then calculates the speed as a function
  // of position, then bounds the speed and positions accoridng to MIN_SPEED, MAX_SPEED,
  // x_min, and x_max, and performs the movement.
  //
  // Speed as a function of position is more difficult to implement compared
  // to simply making speed linear with time, but it makes the operation more robust;
  // if the carriage is blocked for a second, it will not get to the target destination
  // and there will be a spike in acceleration for a bit after it is released.
  // Speed as a function of position makes it possible to easily recover from
  // being stopped for a bit and quickly return to the expected acceleration
  // value without undershooting the target.

  float velocity, x_init, x_final, sign;
  float velocity_init, velocity_fin;

  float deltaV, deltaX, accel, v_init_sqare, time, v_intermediate;

  // Very first thing we do is capture yw so it doesn't change
  x_init = yw;

  // Convert from mm/in to degrees
  // Find the target position, depending on whether we are in absolute or relative mode
  reading_x = interpolate_pos(reading_x);
  if(controller_flag & 1<<POS_ABSOLUTE){
    // If doing absolute positioning, use home as reference point
    reading_x = xmin - reading_x;
  }
  else{
    // Else, add on to current position
    reading_x = x_init - reading_x;
  }

  // Next, figure out which direction we want to go
  sign  = (reading_x > x_init) - (reading_x < x_init);
  // Sign is 0 when equal, 1 when reading_x > x_init, and -1 otherwise.
  // If reading_x > x_init, we want positive velocity and an increase in
  // the x position relative to our target. Otherwise, we want a negative velocity.
  velocity_init = feedrate * sign;
  
  // If we don't have a second feedrate, the final velocity is the same as initial
  if(reading_misc == NOT_FOUND){
    velocity_fin = velocity_init;
  }
  else{
    // If we do have a second feedrate, update the feedrate variable and
    // the final velocity with the value in the command.
    feedrate = interpolate_vel(reading_misc);
    velocity_fin = sign * feedrate;
  }

  // Save our final desitnation, even if it's out of bounds. This lets us
  // accelerate/decellerate correctly for the in-bounds portion of the travel
  x_final = reading_x;
  // Bound the position to stay in bounds when actually moving
  reading_x = bound_pos(reading_x);
  // We can pre-calculate some useful values so we aren't wasting time in the loop
  deltaV = velocity_fin - velocity_init;
  deltaX = x_final - x_init;
  v_init_sqare = velocity_init * velocity_init;
  v_intermediate = (velocity_init) + (deltaV/2.0);
  accel = (deltaV * deltaX)/v_intermediate;
  // Load up the global variables with the pre-calculated values
  target = reading_x;
  // t = (D1)*(D2 + SQRT(D3**2 -D4(D5-yw)))
  // We see we need 5 precomputed global variables
  data1 = (v_intermediate/(deltaV * deltaX)) * sign;
  data2 = -velocity_init*sign;
  data3 = velocity_init;
  data4 = 2*deltaV*v_intermediate/deltaX;
  data5 = x_init;
  data6 = accel;
  dir_going = sign;
  SerialUSB.println(data1);
  SerialUSB.println(data2);
  SerialUSB.println(data3);
  SerialUSB.println(data4);
  SerialUSB.println(data5);
  SerialUSB.println(data6);
  // Go to velocity mode with 0 velocity for now...
  // velocity will soon be updated in the control interrupt
  mode = 'v';
  r = 0;
  // Send command to the control interrupt
  controller_flag |= 1<<BUSY;
  controller_flag |= LINEAR_COMMAND<<COMMAND_SHIFT;
  // We are done here
  return;
}

void process_g(int code, char instruction[], int len){
  float reading_x, reading_misc;          // For managing the readings

  // Clear the command bits
  controller_flag &= ~COMMAND_MASK;

  switch(code){
    case EMPTY:
      SerialUSB.println("Please give a command!");
      break;

    case RAPID_MOV:
      // Move to target point at maximum feedrate
      // This is easy to do so we are handling this case right here.
      reading_x = search_code('X', instruction, len);
      if(reading_x == NOT_FOUND){
        SerialUSB.println("Give a x position");
        return;
      }
      // Convert the reading 
      reading_x = interpolate_pos(reading_x);

      if(controller_flag & 1<<POS_ABSOLUTE){
        // If doing absolute positioning, use home as reference point
        reading_x = xmin - reading_x;
      }
      else{
        // Else, add on to current position
        reading_x = yw - reading_x;
      }
      // Keep output position within boundaries
      mode = 'x';
      controller_flag |= 1<<BUSY;
      controller_flag |= MOVE_COMMAND<<COMMAND_SHIFT;
      r = bound_pos(reading_x);
      break;
    case LINEAR_MOV:
      // First, we handle the case "G1 Fxxx" and set the feedrate to xxx without
      // doing any movement. This occurs when there is no X command.
      reading_misc = search_code('F', instruction, len);
      reading_x = search_code('X', instruction, len);
      if(reading_x == NOT_FOUND){
        // If no x position is given, update the feedrate and return.
        if(reading_misc != NOT_FOUND){
          feedrate = interpolate_vel(reading_misc);
        }
        return;
      }
      // Otherwise, we have something like "G1 Xxxx" or "G1 Xxxx Fxxx"
      // Hop into a function to do this
      linear_move_action(reading_x, reading_misc);
      break;

    case SET_ABS:
      // Set absolute positioning
      controller_flag |= 1<<POS_ABSOLUTE;
      break;

    case SET_REL:  
      // Set relative positioning
      controller_flag &= ~(1<<POS_ABSOLUTE);
      break;

    case CHANGE_UNIT_IN:
      // Change units to inches
      controller_flag &= ~(1<<UNITS_MM);
      break;

    case CHANGE_UNIT_MM:
      // Change units to mm
      controller_flag |= 1<<UNITS_MM;
      break;

    case SET_HOME:
      // Set the current location to home
      xmin = yw;
      break;  
    case HOME:
      // This too is straightforward to implement so we are doing it here.
      // If no parameters are given, calibrate position and go home.
      // If axes are given, home the given axes without calibrating position.
      // Only the x-axis is implemented; search for x
      if(search_code('X', instruction, len) == NOT_FOUND){
        // Do the full calibration
        if(U > UNLOADED_EFFORT_LIM){
          SerialUSB.println("Error: Effort limit exceeded");
          return; // Can't home if effort's too high
        }
        // Run position calibration and home
        calib_home();        
      }
      // Then, if calibrated, move it home at full speed.
      if(xmin != 0 || xmax != 0){
        mode = 'x';
        r = xmin;
      }
      break;
    case DWELL:
      reading_misc = search_code('P', instruction, len);
      if(reading_misc != NOT_FOUND){
        // Set global variables with timing info
        // and the command that we are doing
        target = reading_misc;
        data1 = millis();
        controller_flag |= 1<<BUSY;
        controller_flag |= DWELL_COMMAND<<COMMAND_SHIFT;
      }
      else{
        SerialUSB.println("Give a dwell time");
      }
    break;
    default:
      SerialUSB.println("This hasn't been implemented yet");
  }
  return;
}

void calib_home(){
  // Check if we need to calibrate
  if (lookup[0] == 0 && lookup[128] == 0 && lookup[1024] == 0){
    if(calibrate()==CALIBRATION_FAIL){
      return;
    }
  }
  // Get initial angle calibration (will return to this later)  
  // Move as far "in" as possible before hitting a high-effort region
  // Make this the new 0
  SerialUSB.println("Trying to calibrate");
  mode = 'v';            // Velocity mode
  // Change directions a bit to get the motor moving
  r = -HOMING_SPEED;
  enableTCInterrupts();  // Start moving!
  delay(SETTLE_TIME);
  r = HOMING_SPEED;      // Move to 0 at HOMING_SPEED
  SerialUSB.println("Moving!");

  while(abs(u_roll) < UNLOADED_EFFORT_LIM){
    delayMicroseconds(FILTER_PERIOD_US); // Idle while waiting for limit to be hit
  }
  SerialUSB.println("Near home");

  r = -1*HOMING_SPEED/4;          // Move at quarter HOMING_SPEED
  while(abs(u_roll) > UNLOADED_EFFORT_NOM || abs(u_roll-u_roll_1) > EFFORT_SLOPE_THRESH){
    delayMicroseconds(FILTER_PERIOD_US); // Idle until we reach nominal effort
  }
  r = 0;                          // Stop moving
  SerialUSB.println("At new home");
  SerialUSB.println(yw);
  xmin = yw;
  delay(SETTLE_TIME);             // Wait for the motors to settle down
          
  // Move "out" as far as possible before hitting a high-effort region
  // Make this the new upper bound.
  r = -HOMING_SPEED;
  while(abs(u_roll) < UNLOADED_EFFORT_LIM){
    delayMicroseconds(FILTER_PERIOD_US); // Idle while waiting for limit to be hit
  }
  SerialUSB.println("Near extreme");

  r = 1*HOMING_SPEED/4;          // Move at quarter HOMING_SPEED
  while(abs(u_roll) > UNLOADED_EFFORT_NOM || abs(u_roll-u_roll_1) > EFFORT_SLOPE_THRESH){
    delayMicroseconds(FILTER_PERIOD_US); // Idle until we reach nominal effort
  }
  r = 0;                          // Stop moving
  delay(SETTLE_TIME);             // Wait for the motors to settle down
  SerialUSB.println("At new extreme");
  SerialUSB.println(yw);
  xmax = yw;
  return;
}

void process_m(int code, char instruction[], int len){
  SerialUSB.println("That's a " + String(code) + " mcode");
  return;
}
  
void parameterQuery() {         //print current parameters in a format that can be copied directly in to Parameters.cpp
  SerialUSB.println(' ');
  SerialUSB.println("----Current Parameters-----");
  SerialUSB.println(' ');
  SerialUSB.println(' ');

  SerialUSB.print("volatile float Fs = ");
  SerialUSB.print(Fs, DEC);
  SerialUSB.println(";  //Sample frequency in Hz");
  SerialUSB.println(' ');

  SerialUSB.print("volatile float pKp = ");
  SerialUSB.print(pKp, DEC);
  SerialUSB.println(";      //position mode PID vallues.");
  
  SerialUSB.print("volatile float pKi = ");
  SerialUSB.print(pKi, DEC);
  SerialUSB.println(";");

  SerialUSB.print("volatile float pKd = ");
  SerialUSB.print(pKd, DEC);
  SerialUSB.println(";");
  
  SerialUSB.print("volatile float pLPF = ");
  SerialUSB.print(pLPF, DEC);
  SerialUSB.println(";");

  SerialUSB.println(' ');

  SerialUSB.print("volatile float vKp = ");
  SerialUSB.print(vKp, DEC);
  SerialUSB.println(";      //velocity mode PID vallues.");

  SerialUSB.print("volatile float vKi = ");
  SerialUSB.print(vKi , DEC);
  SerialUSB.println(";");
 // SerialUSB.println(vKi * Fs, DEC);
 // SerialUSB.println(" / Fs;");

  SerialUSB.print("volatile float vKd = ");
  SerialUSB.print(vKd, DEC);
  SerialUSB.println(";");
 // SerialUSB.print(vKd / Fs);
 // SerialUSB.println(" * FS;");
  SerialUSB.print("volatile float vLPF = ");
  SerialUSB.print(vLPF, DEC);
  SerialUSB.println(";");

  SerialUSB.println("");
  SerialUSB.println("//This is the encoder lookup table (created by calibration routine)");
  SerialUSB.println("");
  
  SerialUSB.println("const float __attribute__((__aligned__(256))) lookup[16384] = {");
  for (int i = 0; i < 16384; i++) {
    SerialUSB.print(lookup[i]);
    SerialUSB.print(", ");
  }
  SerialUSB.println("");
  SerialUSB.println("};");

}



void oneStep() {           /////////////////////////////////   oneStep    ///////////////////////////////
  
  if (dir==CCW) {
    stepNumber += 1;
  }
  else {
    stepNumber -= 1;
  }

  //output(1.8 * stepNumber, 64); //updata 1.8 to aps..., second number is control effort
  output(aps * stepNumber, (int)(0.33 * uMAX));
  delay(10);
}

int readEncoder()           //////////////////////////////////////////////////////   READENCODER   ////////////////////////////
{
  long angleTemp;
  
  CHIPSELECT_LOW(); //digitalWrite(chipSelectPin, LOW);

  byte b1 = SPI.transfer(0xFF);
  byte b2 = SPI.transfer(0xFF);

  angleTemp = (((b1 << 8) | b2) & 0B0011111111111111);

  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);
  return angleTemp;
}



void readEncoderDiagnostics()           //////////////////////////////////////////////////////   READENCODERDIAGNOSTICS   ////////////////////////////
{
  long angleTemp;
  CHIPSELECT_LOW(); //digitalWrite(chipSelectPin, LOW);

  ///////////////////////////////////////////////READ DIAAGC (0x3FFC)
  SerialUSB.println("------------------------------------------------");

  SerialUSB.println("Checking AS5047 diagnostic and error registers");
  SerialUSB.println("See AS5047 datasheet for details");
  SerialUSB.println(" ");

  SPI.transfer(0xFF);
  SPI.transfer(0xFC);

  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);
  delay(1);
  CHIPSELECT_LOW();    //digitalWrite(chipSelectPin, LOW);

  byte b1 = SPI.transfer(0xC0);
  byte b2 = SPI.transfer(0x00);

  SerialUSB.print("Check DIAAGC register (0x3FFC) ...  ");
  SerialUSB.println(" ");

  angleTemp = (((b1 << 8) | b2) & 0B1111111111111111);
  SerialUSB.println((angleTemp | 0B1110000000000000000 ), BIN);

  if (angleTemp & (1 << 14))    SerialUSB.println("  Error occurred  ");

  if (angleTemp & (1 << 11))    SerialUSB.println("  MAGH - magnetic field strength too high, set if AGC = 0x00. This indicates the non-linearity error may be increased");

  if (angleTemp & (1 << 10))    SerialUSB.println("  MAGL - magnetic field strength too low, set if AGC = 0xFF. This indicates the output noise of the measured angle may be increased");

  if (angleTemp & (1 << 9))     SerialUSB.println("  COF - CORDIC overflow. This indicates the measured angle is not reliable");

  if (angleTemp & (1 << 8))     SerialUSB.println("  LF - offset compensation completed. At power-up, an internal offset compensation procedure is started, and this bit is set when the procedure is completed");

  if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 11)) | (angleTemp & (1 << 10)) | (angleTemp & (1 << 9))))  SerialUSB.println("Looks good!");
  SerialUSB.println(" ");


  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);
  delay(1);
  CHIPSELECT_LOW();    //digitalWrite(chipSelectPin, LOW);

  SPI.transfer(0x40);
  SPI.transfer(0x01);
  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);

  delay(1);
  CHIPSELECT_LOW();    //digitalWrite(chipSelectPin, LOW);

  b1 = SPI.transfer(0xC0);
  b2 = SPI.transfer(0x00);


  SerialUSB.print("Check ERRFL register (0x0001) ...  ");
  SerialUSB.println(" ");



  angleTemp = (((b1 << 8) | b2) & 0B1111111111111111);
  SerialUSB.println((angleTemp | 0B1110000000000000000 ), BIN);

  if (angleTemp & (1 << 14)) {
    SerialUSB.println("  Error occurred  ");
  }
  if (angleTemp & (1 << 2)) {
    SerialUSB.println("  parity error ");
  }
  if (angleTemp & (1 << 1)) {
    SerialUSB.println("  invalid register  ");
  }
  if (angleTemp & (1 << 0)) {
    SerialUSB.println("  framing error  ");
  }
  if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 2)) | (angleTemp & (1 << 1)) | (angleTemp & (1 << 0))))  SerialUSB.println("Looks good!");

  SerialUSB.println(" ");

  CHIPSELECT_HIGH();   //digitalWrite(chipSelectPin, HIGH);


  delay(1);

}


void print_angle()                ///////////////////////////////////       PRINT_ANGLE   /////////////////////////////////
{
  SerialUSB.print("stepNumber: ");
  SerialUSB.print(stepNumber, DEC);
  SerialUSB.print(" , ");
//  SerialUSB.print(stepNumber * aps, DEC);
//  SerialUSB.print(" , ");
  SerialUSB.print("Angle: ");
  SerialUSB.print(read_angle(), 2);
  SerialUSB.print(", raw encoder: ");
  SerialUSB.print(readEncoder());
  SerialUSB.println();
}


void receiveEvent(int howMany)
{
  while (1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    SerialUSB.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  SerialUSB.println(x);         // print the integer
  r = 0.1 * ((float)x);
}

int mod(int xMod, int mMod) {
  return (xMod % mMod + mMod) % mMod;
}



void setupTCInterrupts() {  // configure the controller interrupt
  // Clear rolling avg filter
  for(int i = 0; i<FILTER_LEN;i++){
    u_past[i] = 0;
  }
  // Start in mm, abs positioning mode
  controller_flag = NO_FLAGS;
  controller_flag |= (1<<UNITS_MM | 1<<POS_ABSOLUTE);
  // Enable GCLK for TC4 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CC[0].reg = (int)( round(48000000 / Fs)); // clock speed / freq = tick check
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC5->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC5->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0

  NVIC_SetPriority(TC5_IRQn, 1);              //Set interrupt priority

  // Enable InterruptVector
  NVIC_EnableIRQ(TC5_IRQn);


  // // Now we also need TC4 for handling the GCode commands
  // while (GCLK->STATUS.bit.SYNCBUSY);
  // TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  // WAIT_TC16_REGS_SYNC(TC4)                      // wait for sync
  // TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  // WAIT_TC16_REGS_SYNC(TC4)
  // TC4->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  // WAIT_TC16_REGS_SYNC(TC4)
  // TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;   // Set perscaler; 46.875kHz
  // WAIT_TC16_REGS_SYNC(TC4)
  // TC4->COUNT16.CC[0].reg = (int)(round(48000000 / (1024*MOVE_CTRL_HZ))); // clock speed / freq = tick check
  // WAIT_TC16_REGS_SYNC(TC4)
  // TC4->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  // TC4->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  // TC4->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0
  // NVIC_SetPriority(TC4_IRQn, 2);              //Set interrupt priority
  // NVIC_EnableIRQ(TC4_IRQn);                  // Enable InterruptVector


  // Enable TC
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  WAIT_TC16_REGS_SYNC(TC5)
  // TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  // WAIT_TC16_REGS_SYNC(TC4)
}

void enableTCInterrupts() {   //enables the controller interrupt ("closed loop mode")
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      //wait for sync
}

void disableTCInterrupts() {  //disables the controller interrupt ("closed loop mode")
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync
}


void antiCoggingCal() {       //This is still under development...  The idea is that we can calibrate out the stepper motor's detent torque by measuring the torque required to hold all possible positions.
  SerialUSB.println(" -----------------BEGIN ANTICOGGING CALIBRATION!----------------");
  mode = 'x';
  r = lookup[1];
  enableTCInterrupts();
  delay(1000);


  for (int i = 1; i < 657; i++) {
    r = lookup[i];
    SerialUSB.print(r, DEC);
    SerialUSB.print(" , ");
    delay(100);
    SerialUSB.println(u, DEC);
  }
  SerialUSB.println(" -----------------REVERSE!----------------");

  for (int i = 656; i > 0; i--) {
    r = lookup[i];
    SerialUSB.print(r, DEC);
    SerialUSB.print(" , ");
    delay(100);
    SerialUSB.println(u, DEC);
  }
  SerialUSB.println(" -----------------DONE!----------------");
  disableTCInterrupts();
}





void parameterEditmain() {

  SerialUSB.println();
  SerialUSB.println("Edit parameters:");
  SerialUSB.println();
  SerialUSB.println("p ----- position loop");
  SerialUSB.println("v ----- velocity loop");
  SerialUSB.println("o ----- other");
  SerialUSB.println("q ----- quit");
  SerialUSB.println();

  while (SerialUSB.available() == 0)  {}
  char inChar2 = (char)SerialUSB.read();

  switch (inChar2) {
    case 'p':
      {
        parameterEditp();
      }
      break;

    case 'v':
      {
        parameterEditv();
      }
      break;

    case 'o':
      {
        parameterEdito();
      }
      break;
    default:
      {}
      break;



  }
}

void parameterEditp() {
  
  bool quit = false;
  while(!quit){
    SerialUSB.println("Edit position loop gains:");
    SerialUSB.println();
    SerialUSB.print("p ----- pKp = ");
    SerialUSB.println(pKp, DEC);
    SerialUSB.print("i ----- pKi = ");
    SerialUSB.println(pKi, DEC);
    SerialUSB.print("d ----- pKd = ");
    SerialUSB.println(pKd, DEC);
    SerialUSB.print("l----- LPF = ");
    SerialUSB.println(pLPF,DEC);
    SerialUSB.println("q ----- quit");
    SerialUSB.println();
    
    while (SerialUSB.available() == 0)  {}
    char inChar3 = (char)SerialUSB.read();
    
    switch (inChar3) {
      case 'p':
        {
          SerialUSB.println("pKp = ?");
          while (SerialUSB.available() == 0)  {}
          pKp = SerialUSB.parseFloat();
          SerialUSB.print("new pKp = ");
          SerialUSB.println(pKp, DEC);
          SerialUSB.println("");
        }
        break;
      case 'i':
        {
          SerialUSB.println("pKi = ?");
          while (SerialUSB.available() == 0)  {}
          pKi = SerialUSB.parseFloat();
          SerialUSB.print("new pKi = ");
          SerialUSB.println(pKi, DEC);
          SerialUSB.println("");
        }
        break;
      case 'd':
        {
          SerialUSB.println("pKd = ?");
          while (SerialUSB.available() == 0)  {}
          pKd = SerialUSB.parseFloat();
          SerialUSB.print("new pKd = ");
          SerialUSB.println(pKd, DEC);
          SerialUSB.println("");
        }
        break;
       case 'l':
        {
          SerialUSB.println("pLPF = ?");
          while (SerialUSB.available() == 0)  {}
          pLPF = SerialUSB.parseFloat();
          pLPFa = exp(pLPF*-2*3.14159/Fs);
          pLPFb = (1.0-pLPFa);
          SerialUSB.print("new pLPF = ");
          SerialUSB.println(pLPF, DEC);
          SerialUSB.println("");
        }
        break;
      case 'q':
        {  
          quit = true;
          SerialUSB.println("");
          SerialUSB.println("done...");
          SerialUSB.println("");
        }
      default:
        {}
        break;
    }
  }
}

void parameterEditv() {
  bool quit = false;
  while(!quit){  
    SerialUSB.println("Edit velocity loop gains:");
    SerialUSB.println();
    SerialUSB.print("p ----- vKp = ");
    SerialUSB.println(vKp, DEC);
    SerialUSB.print("i ----- vKi = ");
    SerialUSB.println(vKi, DEC);
    SerialUSB.print("d ----- vKd = ");
    SerialUSB.println(vKd, DEC);
    SerialUSB.print("l ----- vLPF = ");
    SerialUSB.println(vLPF, DEC);
    SerialUSB.println("q ----- quit");
    SerialUSB.println();
  
    while (SerialUSB.available() == 0)  {}
    char inChar4 = (char)SerialUSB.read();
  
    switch (inChar4) {
      case 'p':
        {
          SerialUSB.println("vKp = ?");
          while (SerialUSB.available() == 0)  {}
          vKp = SerialUSB.parseFloat();
          SerialUSB.print("new vKp = ");
          SerialUSB.println(vKp, DEC);
        }
        break;
      case 'i':
        {
          SerialUSB.println("vKi = ?");
          while (SerialUSB.available() == 0)  {}
          vKi = SerialUSB.parseFloat();
          SerialUSB.print("new vKi = ");
          SerialUSB.println(vKi, DEC);
        }
        break;
      case 'd':
        {
          SerialUSB.println("vKd = ?");
          while (SerialUSB.available() == 0)  {}
          vKd = SerialUSB.parseFloat();
          SerialUSB.print("new vKd = ");
          SerialUSB.println(vKd, DEC);
        }
        break;
       case 'l':
        {
          SerialUSB.println("vLPF = ?");
          while (SerialUSB.available() == 0)  {}
          vLPF = SerialUSB.parseFloat();
          vLPFa = (exp(vLPF*-2*3.14159/Fs));
          vLPFb = (1.0-vLPFa)* Fs * 0.16666667;
          SerialUSB.print("new vLPF = ");
          SerialUSB.println(vLPF, DEC);
          SerialUSB.println("");
        }
        break;
      case 'q':
        {  
          quit = true;
          SerialUSB.println("");
          SerialUSB.println("done...");
          SerialUSB.println("");  
        }
      default:
        {}
        break;
    }
  }  
}

void parameterEdito() {


  SerialUSB.println("Edit other parameters:");
  SerialUSB.println();
  SerialUSB.print("p ----- PA = ");
  SerialUSB.println(PA, DEC);
  SerialUSB.println();


  while (SerialUSB.available() == 0)  {}
  char inChar3 = (char)SerialUSB.read();

  switch (inChar3) {
    case 'p':
      {
        SerialUSB.println("PA = ?");
        while (SerialUSB.available() == 0)  {}
        PA = SerialUSB.parseFloat();
        SerialUSB.print("new PA = ");
        SerialUSB.println(PA, DEC);
      }

      break;
    default:
      {}
      break;
  }
}



void hybridControl() {        //still under development

  static int missed_steps = 0;
  static float iLevel = 0.6;  //hybrid stepping current level.  In this mode, this current is continuous (unlike closed loop mode). Be very careful raising this value as you risk overheating the A4954 driver!
  static float rSense = 0.15;

  if (yw < r - aps) {
    missed_steps -= 1;
  }
  else if (yw > r + aps) {
    missed_steps += 1;
  }

  output(0.1125 * (-(r - missed_steps)), (255 / 3.3) * (iLevel * 10 * rSense));

}

void serialMenu() {
  SerialUSB.println("");
  SerialUSB.println("");
  SerialUSB.println("----- Mechaduino 0.X -----");
  SerialUSB.print("Firmware: ");
  SerialUSB.println(firmware_version);
  SerialUSB.print("Identifier: ");
  SerialUSB.println(identifier);
  SerialUSB.println("");
  SerialUSB.println("Main menu");
  SerialUSB.println("");
  SerialUSB.println(" s  -  step");
  SerialUSB.println(" d  -  dir");
  SerialUSB.println(" p  -  print angle");
  SerialUSB.println("");
  SerialUSB.println(" c  -  write new calibration table");
  SerialUSB.println(" e  -  check encoder diagnositics");
  SerialUSB.println(" q  -  parameter query");
  SerialUSB.println("");
  SerialUSB.println(" x  -  position mode");
  SerialUSB.println(" v  -  velocity mode");
  SerialUSB.println(" t  -  torque mode");
  SerialUSB.println("");
  SerialUSB.println(" y  -  enable control loop");
  SerialUSB.println(" n  -  disable control loop");
  SerialUSB.println(" r  -  enter new setpoint");
  SerialUSB.println("");
   SerialUSB.println(" j  -  step response");
  SerialUSB.println(" k  -  edit controller gains -- note, these edits are stored in volatile memory and will be reset if power is cycled");
  SerialUSB.println(" g  -  generate sine commutation table");
  SerialUSB.println(" m  -  print main menu");
  // SerialUSB.println(" f  -  get max loop frequency");
  SerialUSB.println("");
}
void sineGen() {
  int temp;
     SerialUSB.println("");
     SerialUSB.println("The sineGen() function in Utils.cpp generates a sinusoidal commutation table.");
     SerialUSB.println("You can experiment with different commutation profiles by modifying this table.");
     SerialUSB.println("The below table should be copied into sine_1 in Parameters.cpp.");   
     SerialUSB.println("");
     delay(3000);
     SerialUSB.println("Printing sine look up table:...");
     SerialUSB.println("");
  for (int x = 0; x <= 3600; x++) {
    //temp = round(1024.0 * sin((3.14159265358979 * ((x * 0.1 / 180.0) + 0.25))));
    temp = round(1024.0 * sin((3.14159265358979 * ((x * 0.1 / 180.0) + 0.0))));
   SerialUSB.print(temp);
   SerialUSB.print(", ");  
  }

}

void moveRel(float pos_final,int vel_max, int accel){
  
   //Use this function for slow relative movements in closed loop position mode
   //
   // This function creates a "trapezoidal speed" trajectory (constant accel, and max speed, constant decel);
   // It works pretty well, but it may not be perfect
   // 
   // pos_final is the desired position in degrees
   // vel_max is the max velocity in degrees/second
   // accel is the max accel in degrees/second^2
   //
   //Note that the actual max velocity is limited by the execution speed of all the math below.
   //Adjusting dpos (delta position, or step size) allows you to trade higher speeds for smoother motion
   //Max speed with dpos = 0.225 degrees is about 180 deg/sec
   //Max speed with dpos = 0.45 degrees is about 360 deg/sec 
  
  float pos = 0;
  float dpos = 0.45;  // "step size" in degrees, smaller is smoother, but will limit max speed, keep below stepper step angle
  float vel = 0;      // 
  float vel_1 =0;
  int start = micros(); //for debugging

  float accel_x_dpos = accel*dpos;  // pre calculate
  float dpos_x_1000000 = dpos*1000000.0; // pre calculate

  float pos_remaining = pos_final-pos;
  unsigned long dt =0; 
  unsigned long t = micros();
  unsigned long t_1 = t;

  float r0 = r;  //hold initial setpoint

  // Assume we're decelerating and calculate speed along deceleration profile
  
  while (abs(pos_remaining) >(dpos/2)){  //(may not actually reach exactly so leave some margin
  
    if (pos_remaining > 0)        // clockwise
    vel = sqrt(2.0 * pos_remaining * accel);
    else                      // counter clockwise
    vel = -sqrt(2.0 * -pos_remaining * accel);

    if (vel > vel_1)  // Check if we actually need to accelerate in  clockwise direction
      {

      if (vel_1 == 0)  
        vel = sqrt(2.0 * accel_x_dpos);
      else
        vel = vel_1 + abs(accel_x_dpos/ vel_1);
      if (vel > vel_max)
        vel = vel_max;
      }
    else if (vel < vel_1)
    {
    // Need to accelerate in  counter clockwise direction
    if (vel_1 == 0)
      vel = -sqrt(2.0 * accel_x_dpos);
    else
      vel = vel_1 - abs(accel_x_dpos/ vel_1);
    if (vel < -vel_max)
      vel = -vel_max;
    }
  //  SerialUSB.println(vel);
  
 
  dt = abs(dpos_x_1000000 / vel);
  
    while(t < t_1 + dt) {           //wait calculated dt 
    t = micros();
    }
  
  if (vel > 0)  pos += dpos;        //update setpoint
  else if (vel < 0) pos -= dpos;
  r= r0 + pos;
  
  //SerialUSB.print(micros()-start);
  //SerialUSB.print(" , ");
  
  t_1 = t;  
  vel_1 = vel;
  pos_remaining = pos_final-pos;
  
  }
  r = r0 +pos_final;
  //SerialUSB.print(micros()-start);
  
}




void moveAbs(float pos_final,int vel_max, int accel){
  
   //Use this function for slow absolute movements in closed loop position mode
   //
   // This function creates a "trapezoidal speed" trajectory (constant accel, and max speed, constant decel);
   // It works pretty well, but it may not be perfect
   // 
   // pos_final is the desired position in degrees
   // vel_max is the max velocity in degrees/second
   // accel is the max accel in degrees/second^2
   //
   //Note that the actual max velocity is limited by the execution speed of all the math below.
   //Adjusting dpos (delta position, or step size) allows you to trade higher speeds for smoother motion
   //Max speed with dpos = 0.225 degrees is about 180 deg/sec
   //Max speed with dpos = 0.45 degrees is about 360 deg/sec
  
  float pos = r;
  float dpos = 0.225;  // "step size" in degrees, smaller is smoother, but will limit max speed, keep below stepper step angle
  float vel = 0;      // 
  float vel_1 =0;
 // int start = micros(); //for debugging

  float accel_x_dpos = accel*dpos;  // pre calculate
  float dpos_x_1000000 = dpos*1000000.0; // pre calculate

  float pos_remaining = pos_final-pos;
  unsigned long dt =0; 
  unsigned long t = micros();
  unsigned long t_1 = t;


  // Assume we're decelerating and calculate speed along deceleration profile
  
  while (abs(pos_remaining) >(dpos/2)){  //(may not actually reach exactly so leave some margin
  
    if (pos_remaining > 0)        // clockwise
    vel = sqrt(2.0 * pos_remaining * accel);
    else                      // counter clockwise
    vel = -sqrt(2.0 * -pos_remaining * accel);

    if (vel > vel_1)  // Check if we actually need to accelerate in  clockwise direction
      {

      if (vel_1 == 0)  
        vel = sqrt(2.0 * accel_x_dpos);
      else
        vel = vel_1 + abs(accel_x_dpos/ vel_1);
      if (vel > vel_max)
        vel = vel_max;
      }
    else if (vel < vel_1)
    {
    // Need to accelerate in  counter clockwise direction
    if (vel_1 == 0)
      vel = -sqrt(2.0 * accel_x_dpos);
    else
      vel = vel_1 - abs(accel_x_dpos/ vel_1);
    if (vel < -vel_max)
      vel = -vel_max;
    }
  //  SerialUSB.println(vel);
  
 
  dt = abs(dpos_x_1000000 / vel);
  
    while(t < t_1 + dt) {           //wait calculated dt 
    t = micros();
    }
  
  if (vel > 0)  pos += dpos;        //update setpoint
  else if (vel < 0) pos -= dpos;
  r= pos;
  
  //SerialUSB.print(micros()-start);    //for debugging
  //SerialUSB.print(" , ");
  
  t_1 = t;  
  vel_1 = vel;
  pos_remaining = pos_final-pos;
  
  }
  r = pos_final;
  //SerialUSB.print(micros()-start);
  
}
