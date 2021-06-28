//Contains the declarations for the functions used by the firmware

#ifndef __UTILS_H__
#define __UTIL_H__
  // Defines for ASCII character manipulation
  // Boolean expressions detecting particular characters
  #define IS_NUMBER(ch)  (((int)ch < 58) && ((int)ch > 47))
  #define IS_CAPITAL(ch) (((int)ch < 91) && ((int)ch > 64))
  #define END_OF_LINE(ch)(((char)ch == '\n') || ((char)ch == '\r'))
  // Indicates when a command is not found
  #define NOT_FOUND      4294967295
  // Indicates when a command is empty
  #define EMPTY          4294967294
  // More GCode command defines
  #define HOME           28
  #define RAPID_MOV      0
  #define LINEAR_MOV     1
  #define UNIT_IN        20
  #define UNIT_MM        21
  #define HOME           28
  
  // How many times we should wait to receive a command
  #define DELAY_COUNT    200
  // How long to wait for a command in microseconds 
  #define DELAY_TIME     50
  // Number of characters in a code
  #define CODE_LEN       4
  // Number of characters in a GCode command
  #define COMMAND_SIZE   250

  // Effort limit for detecting end stops
  // These values are for when there is no load on the motor
  // beyond poseidon itself. You will need to tune these values.
  #define UNLOADED_EFFORT_LIM     40
  #define UNLOADED_EFFORT_NOM     20
  // Speed at which to do homing, revolutions per minute
  #define HOMING_SPEED   60

  // Time it takes for vibrations to stop, milliseconds
  #define SETTLE_TIME    150

	void setupPins();                 // initializes pins
	
	void setupSPI();                  //initializes SPI

  void configureStepDir();          //configure step/dir interface
  
  void configureEnablePin();        //configure enable pin 
		
	void stepInterrupt();             //step interrupt handler

  void dirInterrupt();              //dir interrupt handler

  void enableInterrupt();           //enable pin interrupt handler

	void output(float theta, int effort);	  //calculates phase currents (commutation) and outputs to Vref pins

	void calibrate();	                //calibration routine
		
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

  double search_code(char key, char instruction[], int string_size);

  void process_g(int code, char instruction[], int len);

  void process_m(int code, char instruction[], int len);
#endif
