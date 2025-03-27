//Contains the declarations for the functions used by the firmware

#ifndef __UTILS_H__
#define __UTIL_H__
  // Defines for ASCII character manipulation
  // Boolean expressions detecting particular characters
  #define IS_NUMBER(ch)  ((((int)ch < 58) && ((int)ch > 47)) || ((int)ch == 46) || ((int)ch == 45))
  #define IS_CAPITAL(ch) (((int)ch < 91) && ((int)ch > 64))
  #define END_OF_LINE(ch)(((char)ch == '\n') || ((char)ch == '\r'))
  // Indicates when a command is not found
  #define NOT_FOUND      888888
  // Indicates when a command is empty
  #define EMPTY          999999
  // More GCode command defines
  #define HOME           28
  #define RAPID_MOV      0
  #define LINEAR_MOV     1
  #define CHANGE_UNIT_IN 20
  #define CHANGE_UNIT_MM 21
  #define DWELL          4
  #define SET_HOME       92
  #define SET_ABS        90
  #define SET_REL        91
  // M code defines
  #define DEBUG          111
  #define ESTOP          112
  
  // How many times we should wait to receive a command
  #define DELAY_COUNT    200
  // How long to wait for a command in microseconds 
  #define DELAY_TIME     50
  // Number of characters in a code (i.e. instruction or len to move)
  #define CODE_LEN       32
  // Number of characters in a GCode command
  #define COMMAND_SIZE   250

  // Effort limit for detecting end stops
  // These values are for when there is no load on the motor
  // beyond poseidon itself. You will need to tune these values.
  #define UNLOADED_EFFORT_LIM     35
  #define UNLOADED_EFFORT_NOM     25
  #define EFFORT_SLOPE_THRESH     1
  // Speed at which to do homing, revolutions per minute
  #define HOMING_SPEED   60

  // Time it takes for vibrations to stop, milliseconds
  #define SETTLE_TIME    500
  #define QUICK_SETTLE   20
  // Time it takes for the encoder readint to stabilize, milliseconds
  #define READ_TIME      10

  // Rotation directions
  #define CW             true
  #define CCW            false

  // Calibration flags
  #define CALIBRATION_FAIL    1
  #define CALIBRATION_SUCCESS 0

  // Rod thread - millimeters per rotation
  #define MM_PER_ROT          1.0
  #define IN_PER_ROT          0.0787402f
  #define DEFAULT_SPEED       10.0  // 10 RPM
  #define MIN_SPEED           0.0005// Measured in RPM
  #define MAX_SPEED           310.0 // measured in RPM

	void setupPins();                 // initializes pins
	
	void setupSPI();                  //initializes SPI

  void configureStepDir();          //configure step/dir interface
  
  void configureEnablePin();        //configure enable pin 
		
	void stepInterrupt();             //step interrupt handler

  void dirInterrupt();              //dir interrupt handler

  void enableInterrupt();           //enable pin interrupt handler

	void output(float theta, int effort);	  //calculates phase currents (commutation) and outputs to Vref pins

	int  calibrate();	                //calibration routine
		
	void serialCheck();               //checks serial port for commands.  Must include this in loop() for serial interface to work

	void parameterQuery();            //Prints current parameters
	
	void oneStep(void);               //take one step
		
	int readEncoder();                //read raw encoder position
	  
	void readEncoderDiagnostics();    //check encoder diagnostics registers
		
	void print_angle();               //for debigging purposes in open loop mode:  prints [step number] , [encoder reading]
	
	void receiveEvent(int howMany);   //for i2c interface...
	
	int mod(int xMod, int mMod);      //modulo, handles negative values properly    
	
	void setupTCInterrupts();         //configures control loop interrupt
	
	void enableTCInterrupts();        //enables control loop interrupt.  Use this to enable "closed-loop" modes
	
	void disableTCInterrupts();       //disables control loop interrupt.  Use this to diable "closed-loop" mode
	
	void antiCoggingCal();            //under development...
	
	void parameterEditmain();         //parameter editing menu
	
	void parameterEditp();            //parameter editing menu
	
	void parameterEditv();            //parameter editing menu
	
	void parameterEdito();            //parameter editing menu

  void hybridControl();             //open loop stepping, but corrects for missed steps.  under development

  void serialMenu();                //main menu
  
  void sineGen();                   //generates sinusoidal commutation table. you can experiment with other commutation profiles 

  void stepResponse();              //generates position mode step response in Serial Plotter

  void moveRel(float pos_final,int vel_max, int accel);     // Generates trapezoidal motion profile for closed loop position mode
  
  void moveAbs(float pos_final,int vel_max, int accel);     // Generates trapezoidal motion profile for closed loop position mode

  void gcode_parse();

  void process_string(char instruction[], int len);

  float search_code(char key, char instruction[], int string_size);

  void process_g(int code, char instruction[], int len);

  void process_m(int code, char instruction[], int len);

  void calib_home();

  void findijStart(int readings[], int* istart, int* jstart);

  float interpolate_pos(float target);

  float interpolate_vel(float target);

  float bound_pos(float target);

  float bound_vel(float speed, float sign);

  void linear_move_action(float reading_x, float reading_misc);

#endif
