#ifndef PARAMETERS_H
#define PARAMETERS_H

// NO RS485/EXTRUDER CONTROLLER SUPPORT
// PLEASE VERIFY PIN ASSIGNMENTS FOR YOUR CONFIGURATION!!!!!!!
#define MOTHERBOARD 5 // ATMEGA168 = 0, SANGUINO = 1, MOTHERBOARD = 2, MEGA/RAMPS = 3, ATMEGA328 = 4, Gen6 = 5, Sanguinololu = 6

//Comment out to disable SD support
//#define SDSUPPORT 1

//Min step delay in microseconds. If you are experiencing missing steps, try to raise the delay microseconds, but be aware this
//Enabled for Gen6.  Leave at default 1

#define STEP_DELAY_MICROS 1

//Step delay over interval ratio. If you are still experiencing missing steps, try to uncomment the following line, but be aware this
//Enabled for Gen6.  leave at default 0.25  . Speeds above 120mm/s are drastically slowed down when this is enabled...  
#define STEP_DELAY_RATIO 0.25

//Comment this to disable ramp acceleration.  
//Ramp Accleration gives you smoother movements and a more quiet machine.  
#define RAMP_ACCELERATION 1  // different maximum feedrates do not require different settings.

//Uncomment this to enable exponential acceleration.  Very hard acceleartion alogarithm.
//#define EXP_ACCELERATION 1  // works good if you have dialed in your settings and you are not changing your feedrates.  
//different maximum feedrates require different settings..  Otherwise inefficient or too fast.


//Acceleration settings
#ifdef RAMP_ACCELERATION
float min_units_per_second = 30.0; // the minimum feedrate, this should be about the speed you are printing your perimeters at.
long max_acceleration_units_per_sq_second = 1000; // Max acceleration in mm/s^2 for printing moves. Higher means faster acceleration. 
long max_travel_acceleration_units_per_sq_second = 1000; // Max acceleration in mm/s^2 for travel moves. Higher means faster acceleration. 
#endif
#ifdef EXP_ACCELERATION
float full_velocity_units = 5; // the units between minimum and G1 move feedrate  
// rule of thumb: full_velocity_units=  sqrt(desired maximum feedrate - min_units_per_second)
float travel_move_full_velocity_units = 10; // used for travel moves.  Use same formula for this..

float min_units_per_second = 35.0; // the minimum feedrate.  Chhose it as about 70% of your fastest "safe" printing speed.
float min_constant_speed_units = 2; // the minimum units of an accelerated move that must be done at constant speed
                                    // Note that if the move is shorter than this value, acceleration won't be perfomed,
                                    // but will be done at the minimum between min_units_per_seconds and move feedrate speeds.
#endif





// AD595 THERMOCOUPLE SUPPORT UNTESTED... USE WITH CAUTION!!!!

//PID settings:
//These settings work very nice with the Mendel parts hotend. (6 Ohm aluminium block)  Leave PID enabled.
#define PIDTEMP 1
#ifdef PIDTEMP
#define PID_MAX 255 // limits current to nozzle
#define PID_INTEGRAL_DRIVE_MAX 255
#define PID_PGAIN 500//100 is 1.0
#define PID_IGAIN 95 //100 is 1.0
#define PID_DGAIN 60 //100 is 1.0
#endif

//Experimental temperature smoothing - only uncomment this if your temp readings are noisy.  Absolutely Necessary for Gen6
#define SMOOTHING 1
#define SMOOTHFACTOR 16 //best to use a power of two here - determines how many values are averaged together by the smoothing algorithm

//Experimental watchdog and minimal temp
//The watchdog waits for the watchperiod in milliseconds whenever an M104 or M109 increases the target temperature
//If the temperature has not increased at the end of that period, the target temperature is set to zero. It can be reset with another M104/M109
//#define WATCHPERIOD 5000 //5 seconds
//The minimal temperature defines the temperature below which the heater will not be enabled.  Setting it under roomtemperature works as failsafe if the circuit breaks.
//#define MINTEMP 10

// Select one of these only to define how the nozzle temp is read.
#define HEATER_USES_THERMISTOR
//#define HEATER_USES_AD595
//#define HEATER_USES_MAX6675

// Select one of these only to define how the bed temp is read.
#define BED_USES_THERMISTOR
//#define BED_USES_AD595


// Calibration formulas
// e_extruded_steps_per_mm = e_feedstock_steps_per_mm * (desired_extrusion_diameter^2 / feedstock_diameter^2)
// new_axis_steps_per_mm = previous_axis_steps_per_mm * (test_distance_instructed/test_distance_traveled)
// units are in millimeters or whatever length unit you prefer: inches,football-fields,parsecs etc

//Calibration variables..  Settings are the default mendel-parts FiveD values.  Feel free to calibrate and correct.
float x_steps_per_unit = 40; 
float y_steps_per_unit = 40;
float z_steps_per_unit = 3333.592;
float e_steps_per_unit = 442.3027; // my settings for Skeinforge 40 and higher.
float max_feedrate = 200000;  // The feedrate the FW will set as upper limit.  Anything higher will be valued down to this. (mm/min)
float max_z_feedrate = 80; // with this limit in skeinforge can be disabled. (mm/min)



//float x_steps_per_unit = 10.047;
//float y_steps_per_unit = 10.047;
//float z_steps_per_unit = 833.398;
//float e_steps_per_unit = 0.706;
//float max_feedrate = 3000;

//For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
const bool X_ENABLE_ON = 0;
const bool Y_ENABLE_ON = 0;
const bool Z_ENABLE_ON = 0;
const bool E_ENABLE_ON = 0;

//Disables axis when it's not being used.
const bool DISABLE_X = false;  
const bool DISABLE_Y = false;
const bool DISABLE_Z = true;  //the only one that is used so infrequently that it is worth disabling. 
const bool DISABLE_E = false;

const bool INVERT_X_DIR = false;// set to true if you are using x axis belt teeth turned to the outside. 
const bool INVERT_Y_DIR = true;
const bool INVERT_Z_DIR = false;
const bool INVERT_E_DIR = false;

//Thermistor settings:

//Uncomment for 100k thermistor
//#include "ThermistorTable_100k.h"
//#include "BedThermistorTable_100k.h"

//Uncomment for 200k thermistor
//#include "ThermistorTable_200k.h"
//#include "BedThermistorTable_200k.h"

//Identical thermistors on heater and bed - use this if you have no heated bed or if the thermistors are the same on both:
//#include "ThermistorTable_200k.h"
//#include "ThermistorTable_100k.h"
#include "ThermistorTable_mendelparts.h"
#define BNUMTEMPS NUMTEMPS
#define bedtemptable temptable

//Endstop Settings
#define ENDSTOPPULLUPS 1
const bool ENDSTOPS_INVERTING = true;  // set to false if using optos from before DEC15, 2010
const bool min_software_endstops = false; //If true, axis won't move to coordinates less than zero.
const bool max_software_endstops = true;  //If true, axis won't move to coordinates greater than the defined lengths below.
const int X_MAX_LENGTH = 200;
const int Y_MAX_LENGTH = 200;
const int Z_MAX_LENGTH = 100;

#define BAUDRATE 57600  // Do not forget to change the setting in Repsnapper.  




#endif
