## GCode Control for Mechaduino

This project is a fork of the original Mechaduino project, forked on 6/20/2021. 
The goal of this project is to replace the Mechaduino default serial commands 
with Gcode commands, particularly for use with 
[poseidon](https://github.com/pachterlab/poseidon).

## Motivation

Using the Mechaduino as our motor controller offers many advantages, especially when used for poseidon. 

1. **More precise positioning:** Feedback control can detect and compensate for
errors in position, ensuring the correct amount of fluid has been deposited.
2. **Torque readings:** Mechaduino gives "effort" readings which are related
to the torque on the stepper motor. This lets us calculate the pressure in
the syringe and find when the pump is at its limits without needing extra
hardware.
3. **All-in-one:** While the Mechaduino advertises itself as a motor controller,
the Mechaduino firmware uses roughly half of the program space and leaves plenty
of RAM available. Thus, with some optimizations, all the code can run on the 
Mechaduino itself without needing another microcontroller to parse serial 
commands.

## Progress

This project is just getting started! Follow along to watch me meet these
fun and exciting milestones:

1. Remove everything that is not necessary from the Mechaduino firmware
2. Implement a GCode parser for single-axis control
3. Implement logging and error handeling (e.g. logging syringe pressure and 
raising an error when torque exceeds some upper bound)
4. Replace the serial GCode commands with I2C GCode commands. Multiple I2C
devices cna connect to the same port, but the same is not true for serial.
5. Implement a serial-controlled I2C host which receives multi-axis GCode
commands and sends the signals to multiple Mechaduinos.