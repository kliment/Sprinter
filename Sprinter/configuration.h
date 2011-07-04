#ifndef PARAMETERS_H
#define PARAMETERS_H

// NO RS485/EXTRUDER CONTROLLER SUPPORT
// PLEASE VERIFY PIN ASSIGNMENTS FOR YOUR CONFIGURATION!!!!!!!
#define MOTHERBOARD 3 // ATMEGA168 = 0, SANGUINO = 1, MOTHERBOARD = 2, MEGA/RAMPS = 3, ATMEGA328 = 4, Gen6 = 5, Sanguinololu = 6

//Comment out to disable SD support
#define SDSUPPORT 1

//Min step delay in microseconds. If you are experiencing missing steps, try to raise the delay microseconds, but be aware this
// If you enable this, make sure STEP_DELAY_RATIO is disabled.
//#define STEP_DELAY_MICROS 1

//Step delay over interval ratio. If you are still experiencing missing steps, try to uncomment the following line, but be aware this
//If you enable this, make sure STEP_DELAY_MICROS is disabled.
//#define STEP_DELAY_RATIO 0.25

//Comment this to disable ramp acceleration
#define RAMP_ACCELERATION 1

//Acceleration settings
#ifdef RAMP_ACCELERATION
//X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
float max_start_speed_units_per_second[] = {25.0,25.0,0.2,10.0};
long max_acceleration_units_per_sq_second[] = {1000,1000,50,10000}; // X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
long max_travel_acceleration_units_per_sq_second[] = {500,500,50}; // X, Y, Z max acceleration in mm/s^2 for travel moves
#endif

// AD595 THERMOCOUPLE SUPPORT UNTESTED... USE WITH CAUTION!!!!

//PID settings:
//Uncomment the following line to enable PID support. This is untested and could be disastrous. Be careful.
//#define PIDTEMP 1
#ifdef PIDTEMP
#define PID_MAX 255 // limits current to nozzle
#define PID_INTEGRAL_DRIVE_MAX 220
#define PID_PGAIN 180 //100 is 1.0
#define PID_IGAIN 2 //100 is 1.0
#define PID_DGAIN 100 //100 is 1.0
#endif

//How often should the heater check for new temp readings, in milliseconds
#define HEATER_CHECK_INTERVAL 500
#define BED_CHECK_INTERVAL 5000
//Comment the following line to enable heat management during acceleration
#define DISABLE_CHECK_DURING_ACC
#ifndef DISABLE_CHECK_DURING_ACC
  //Uncomment the following line to disable heat management during the move
  //#define DISABLE_CHECK_DURING_MOVE
#endif
//Uncomment the following line to disable heat management during travel moves (and extruder-only moves, eg: retracts), strongly recommended if you are missing steps mid print.
//Probably this should remain commented if are using PID.
//It also defines the max milliseconds interval after which a travel move is not considered so for the sake of this feature.
#define DISABLE_CHECK_DURING_TRAVEL 1000

//Experimental temperature smoothing - only uncomment this if your temp readings are noisy
//#define SMOOTHING 1
//#define SMOOTHFACTOR 16 //best to use a power of two here - determines how many values are averaged together by the smoothing algorithm

//Experimental watchdog and minimal temp
//The watchdog waits for the watchperiod in milliseconds whenever an M104 or M109 increases the target temperature
//If the temperature has not increased at the end of that period, the target temperature is set to zero. It can be reset with another M104/M109
//#define WATCHPERIOD 5000 //5 seconds
//The minimal temperature defines the temperature below which the heater will not be enabled
//#define MINTEMP 

//Experimental max temp
//When temperature exceeds max temp, your bot will halt.
//This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
//You should use MINTEMP for thermistor short/failure protection.
//#define MAXTEMP 275

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

//Calibration variables
const int NUM_AXIS = 4; // The axis order in all axis related arrays is X, Y, Z, E
bool axis_relative_modes[] = {false, false, false, false};
float axis_steps_per_unit[] = {80.376,80.376,3200/1.25,16}; // {X steps per unit, Y steps per unit, Z steps per unit, E steps per unit}
//For SAE Prusa mendeel float z_steps_per_unit = should be 3200/1.411 for 5/16-18 rod and 3200/1.058 for 5/16-24
//float axis_steps_per_unit[] = {10.047,10.047,833.398,0.706};
float max_feedrate[] = {200000, 200000, 240, 500000}; //mmm, acceleration!

//For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
const bool X_ENABLE_ON = 0;
const bool Y_ENABLE_ON = 0;
const bool Z_ENABLE_ON = 0;
const bool E_ENABLE_ON = 0;

//Disables axis when it's not being used.
const bool DISABLE_X = false;
const bool DISABLE_Y = false;
const bool DISABLE_Z = true;
const bool DISABLE_E = false;

const bool INVERT_X_DIR = false;
const bool INVERT_Y_DIR = false;
const bool INVERT_Z_DIR = true;
const bool INVERT_E_DIR = false;

// Sets direction of endstops when homing; 1=MAX, -1=MIN
const int X_HOME_DIR = -1;
const int Y_HOME_DIR = -1;
const int Z_HOME_DIR = -1;


//Thermistor settings:

//Uncomment for 100k thermistor
//#include "ThermistorTable_100k.h"
//#include "BedThermistorTable_100k.h"

//Uncomment for 200k thermistor
//#include "ThermistorTable_200k.h"
//#include "BedThermistorTable_200k.h"

//Identical thermistors on heater and bed - use this if you have no heated bed or if the thermistors are the same on both:
#include "ThermistorTable_200k.h"
//#include "ThermistorTable_100k.h"
//#include "ThermistorTable_mendelparts.h"
#define BNUMTEMPS NUMTEMPS
#define bedtemptable temptable

//Endstop Settings
#define ENDSTOPPULLUPS 1
const bool ENDSTOPS_INVERTING = false;
const bool min_software_endstops = false; //If true, axis won't move to coordinates less than zero.
const bool max_software_endstops = true;  //If true, axis won't move to coordinates greater than the defined lengths below.
const int X_MAX_LENGTH = 220;
const int Y_MAX_LENGTH = 220;
const int Z_MAX_LENGTH = 100;

#define BAUDRATE 115200

//Uncomment the following line to enable debugging. You can better control debugging below the following line
//#define DEBUG
#ifdef DEBUG
  //#define DEBUG_PREPARE_MOVE //Enable this to debug prepare_move() function
  //#define DEBUG_BRESENHAM //Enable this to debug the Bresenham algorithm
  //#define DEBUG_RAMP_ACCELERATION //Enable this to debug all constant acceleration info
  //#define DEBUG_MOVE_TIME //Enable this to time each move and print the result
  //#define DEBUG_HEAT_MGMT //Enable this to debug heat management. WARNING, this will cause axes to jitter!
  //#define DEBUG_DISABLE_CHECK_DURING_TRAVEL //Debug the namesake feature, see above in this file
#endif

#endif
