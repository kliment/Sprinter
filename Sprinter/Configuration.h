#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// BASIC SETTINGS: select your board type, thermistor type, axis scaling, and endstop configuration

//// The following define selects which electronics board you have. Please choose the one that matches your setup
// MEGA/RAMPS up to 1.2  = 3,
// RAMPS 1.3 = 33
// Gen6 = 5, 
// Sanguinololu up to 1.1 = 6
// Sanguinololu 1.2 and above = 62
// Teensylu (at90usb) = 8
// Gen 3 Plus = 21
// gen 3  Monolithic Electronics = 22
// Gen3 PLUS for TechZone Gen3 Remix Motherboard = 23
#define MOTHERBOARD 3 

//// Thermistor settings:
// 1 is 100k thermistor
// 2 is 200k thermistor
// 3 is mendel-parts thermistor
// 4 is 10k thermistor
// 5 is ParCan supplied 104GT-2 100K
// 6 is EPCOS 100k
// 7 is 100k Honeywell thermistor 135-104LAG-J01
#define THERMISTORHEATER 1
#define THERMISTORBED 1

//// Calibration variables
// X, Y, Z, E steps per unit - Metric Prusa Mendel with Wade extruder:
float axis_steps_per_unit[] = {80, 80, 3200/1.25,700}; 
// Metric Prusa Mendel with Makergear geared stepper extruder:
//float axis_steps_per_unit[] = {80,80,3200/1.25,1380}; 
// MakerGear Hybrid Prusa Mendel:
// Z axis value is for .9 stepper(if you have 1.8 steppers for Z, you need to use 2272.7272)
//float axis_steps_per_unit[] = {104.987, 104.987, 4545.4544, 1487};

//// Endstop Settings
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors
// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
//If your axes are only moving in one direction, make sure the endstops are connected properly.
//If your axes move in one direction ONLY when the endstops are triggered, set [XYZ]_ENDSTOP_INVERT to true here:
const bool X_ENDSTOP_INVERT = false;
const bool Y_ENDSTOP_INVERT = false;
const bool Z_ENDSTOP_INVERT = false;

// This determines the communication speed of the printer
#define BAUDRATE 115200

// Comment out (using // at the start of the line) to disable SD support:
#define SDSUPPORT


//// ADVANCED SETTINGS - to tweak parameters

#include "thermistortables.h"

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0

// Disables axis when it's not being used.
const bool DISABLE_X = false;
const bool DISABLE_Y = false;
const bool DISABLE_Z = true;
const bool DISABLE_E = false;

// Inverting axis direction
const bool INVERT_X_DIR = false;
const bool INVERT_Y_DIR = false;
const bool INVERT_Z_DIR = true;
const bool INVERT_E_DIR = false;

//// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

const bool min_software_endstops = false; //If true, axis won't move to coordinates less than zero.
const bool max_software_endstops = true;  //If true, axis won't move to coordinates greater than the defined lengths below.
const int X_MAX_LENGTH = 200;
const int Y_MAX_LENGTH = 200;
const int Z_MAX_LENGTH = 100;

//// MOVEMENT SETTINGS
const int NUM_AXIS = 4; // The axis order in all axis related arrays is X, Y, Z, E
float max_feedrate[] = {200000, 200000, 240, 500000};
float homing_feedrate[] = {1500,1500,120};
bool axis_relative_modes[] = {false, false, false, false};

// Min step delay in microseconds. If you are experiencing missing steps, try to raise the delay microseconds, but be aware this
// If you enable this, make sure STEP_DELAY_RATIO is disabled.
//#define STEP_DELAY_MICROS 1

// Step delay over interval ratio. If you are still experiencing missing steps, try to uncomment the following line, but be aware this
// If you enable this, make sure STEP_DELAY_MICROS is disabled. (except for Gen6: both need to be enabled.)
//#define STEP_DELAY_RATIO 0.25

///Oscillation reduction.  Forces x,y,or z axis to be stationary for ## ms before allowing axis to switch direcitons.  Alternative method to prevent skipping steps.  Uncomment the line below to activate.
//#define RAPID_OSCILLATION_REDUCTION
#ifdef RAPID_OSCILLATION_REDUCTION
long min_time_before_dir_change = 30; //milliseconds
#endif

// Comment this to disable ramp acceleration
#define RAMP_ACCELERATION

//// Acceleration settings
#ifdef RAMP_ACCELERATION
// X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
float max_start_speed_units_per_second[] = {25.0,25.0,0.2,10.0};
long max_acceleration_units_per_sq_second[] = {1000,1000,50,10000}; // X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
long max_travel_acceleration_units_per_sq_second[] = {500,500,50,500}; // X, Y, Z max acceleration in mm/s^2 for travel moves
#endif

// Machine UUID
// This may be useful if you have multiple machines and wish to identify them by using the M115 command. 
// By default we set it to zeros.
char uuid[] = "00000000-0000-0000-0000-000000000000";


//// AD595 THERMOCOUPLE SUPPORT UNTESTED... USE WITH CAUTION!!!!

//// PID settings:
// Uncomment the following line to enable PID support. This is untested and could be disastrous. Be careful.
//#define PIDTEMP 1
#ifdef PIDTEMP
#define PID_INTEGRAL_DRIVE_MAX 80 // too big, and heater will lag after changing temperature, too small and it might not compensate enough for long-term errors
#define PID_PGAIN 2560 //256 is 1.0  // value of X means that error of 1 degree is changing PWM duty by X, probably no need to go over 25
#define PID_IGAIN 64 //256 is 1.0  // value of X (e.g 0.25) means that each degree error over 1 sec (2 measurements) changes duty cycle by 2X (=0.5) units (verify?)
#define PID_DGAIN 4096 //256 is 1.0  // value of X means that around reached setpoint, each degree change over one measurement (half second) adjusts PWM by X units to compensate
// magic formula 1, to get approximate "zero error" PWM duty. Take few measurements with low PWM duty and make linear fit to get the formula
#define HEATER_DUTY_FOR_SETPOINT(setpoint) ((int)((187L*(long)setpoint)>>8)-27)  // for my makergear hot-end: linear fit {50,10},{60,20},{80,30},{105,50},{176,100},{128,64},{208,128}
// magic formula 2, to make led brightness approximately linear
#define LED_PWM_FOR_BRIGHTNESS(brightness) ((64*brightness-1384)/(300-brightness))
#endif

// Change this value (range 1-255) to limit the current to the nozzle
#define HEATER_CURRENT 255

// How often should the heater check for new temp readings, in milliseconds
#define HEATER_CHECK_INTERVAL 500
#define BED_CHECK_INTERVAL 5000
// Comment the following line to enable heat management during acceleration
#define DISABLE_CHECK_DURING_ACC
#ifndef DISABLE_CHECK_DURING_ACC
  // Uncomment the following line to disable heat management during moves
  //#define DISABLE_CHECK_DURING_MOVE
#endif
// Uncomment the following line to disable heat management during travel moves (and extruder-only moves, eg: retracts), strongly recommended if you are missing steps mid print.
// Probably this should remain commented if are using PID.
// It also defines the max milliseconds interval after which a travel move is not considered so for the sake of this feature.
#define DISABLE_CHECK_DURING_TRAVEL 1000

//// Temperature smoothing - only uncomment this if your temp readings are noisy (Gen6 without EvdZ's 5V hack)
//#define SMOOTHING
//#define SMOOTHFACTOR 16 //best to use a power of two here - determines how many values are averaged together by the smoothing algorithm

//// Experimental watchdog and minimal temp
// The watchdog waits for the watchperiod in milliseconds whenever an M104 or M109 increases the target temperature
// If the temperature has not increased at the end of that period, the target temperature is set to zero. It can be reset with another M104/M109
//#define WATCHPERIOD 5000 //5 seconds

// Actual temperature must be close to target for this long before M109 returns success
//#define TEMP_RESIDENCY_TIME 20  // (seconds)
//#define TEMP_HYSTERESIS 5       // (C°) range of +/- temperatures considered "close" to the target one

//// The minimal temperature defines the temperature below which the heater will not be enabled
#define MINTEMP 5

//// Experimental max temp
// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define MAXTEMP 275

// Select one of these only to define how the nozzle temp is read.
#define HEATER_USES_THERMISTOR
//#define HEATER_USES_AD595
//#define HEATER_USES_MAX6675

// Select one of these only to define how the bed temp is read.
#define BED_USES_THERMISTOR
//#define BED_USES_AD595

//This is for controlling a fan to cool down the stepper drivers
//it will turn on when any driver is enabled
//and turn off after the set amount of seconds from last driver being disabled again
//#define CONTROLLERFAN_PIN 23 //Pin used for the fan to cool controller, comment out to disable this function
#define CONTROLLERFAN_SEC 60 //How many seconds, after all motors were disabled, the fan should run

// Uncomment the following line to enable debugging. You can better control debugging below the following line
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
