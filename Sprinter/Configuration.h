#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// By using a custom config file here, we keep this main Configuration.h
// file intact, making it MUCH easier to update.
// We also keep the original values untouch, as a quick reference. 
// To use your own custom config, uncomment the line below, modifying
// "customConfigFile.h" with your own config file name. 
// To create your own custom one, just copy/paste this one into your new file.
//#include "customConfigFile.h"

// If MOTHERBOARD is already defined from without a custom config, 
// then ignore the values below
#ifndef MOTHERBOARD
    // BASIC SETTINGS: select your board type, thermistor type, axis scaling, and endstop configuration

    //// The following define selects which electronics board you have. Please choose the one that matches your setup
    // MEGA/RAMPS up to 1.2  = 3,
    // RAMPS 1.3 = 33
    // Gen6 = 5, 
    // Sanguinololu up to 1.1 = 6
    // Sanguinololu 1.2 and above = 62
    // Gen 3 Plus = 21
    // gen 3  Monolithic Electronics = 22
    #define MOTHERBOARD 3 

    //// Thermistor settings:
    // 1 is 100k thermistor
    // 2 is 200k thermistor
    // 3 is mendel-parts thermistor
    // 4 is 10k thermistor
    // 5 is ParCan supplied 104GT-2 100K
    // 6 is EPCOS 100k
    // 7 is Honeywell 100K Thermistor (135-104LAG-J01)
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
    #define ENDSTOPPULLUPS 1 // Comment this out (using // at the start of the line) to disable the endstop pullup resistors
    // The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
    const bool ENDSTOPS_INVERTING = false; //set to true to invert the logic of the endstops
    //If your axes are only moving in one direction, make sure the endstops are connected properly.
    //If your axes move in one direction ONLY when the endstops are triggered, set ENDSTOPS_INVERTING to true here

    // This determines the communication speed of the printer
    #define BAUDRATE 115200

    // Comment out (using // at the start of the line) to disable SD support:
    #define SDSUPPORT 1


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

    // Comment this to disable ramp acceleration
    #define RAMP_ACCELERATION 1

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
    #define PID_MAX 255 // limits current to nozzle
    #define PID_INTEGRAL_DRIVE_MAX 220
    #define PID_PGAIN 180 //100 is 1.0
    #define PID_IGAIN 2 //100 is 1.0
    #define PID_DGAIN 100 //100 is 1.0
    #endif

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
    //#define SMOOTHING 1
    //#define SMOOTHFACTOR 16 //best to use a power of two here - determines how many values are averaged together by the smoothing algorithm

    //// Experimental watchdog and minimal temp
    // The watchdog waits for the watchperiod in milliseconds whenever an M104 or M109 increases the target temperature
    // If the temperature has not increased at the end of that period, the target temperature is set to zero. It can be reset with another M104/M109
    //#define WATCHPERIOD 5000 //5 seconds

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


    //=========================================================
    // custom setup and loop example
    //
    // this is a test example of how to use the custom
    // setup code and loop code feature. 
    // 
    // by defining the CUSTOM_SETUP/CUSTOM_LOOP macro as the name of your
    // function, the main setup()/loop() function of the firmware
    // will execute your custom code right at the bottom. 
    // 
    // very efficient way to write custom code of your own 
    // that gets hooked into the firmware without ANY modifying to the 
    // original code, making it a snap to keep your custom 
    // firmware up-to-date with the main depot!
    //
    // Your custom code should be defined in a custom config file, 
    // 
    //=========================================================

    // the line below is commented out since 
    // this custom code example is disabled by default
    //#define CUSTOM_SETUP setupMicrosteppingAndLCD 
    //#define CUSTOM_LOOP  loopLcdUpdate
    
    #ifdef CUSTOM_SETUP
        // include the arduino liquid crystal library, only if 
        // custom setup is enabled!
        #include <LiquidCrystal.h>
        
        // also, create the main LCD global variable.
        LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
        
        // custom setup function code 
        void setupMicrostepping()
        {
            
            // In my custom ramps board, I connected pins 22,24 and 23 to 
            // pololu MS1,2 and 3. That way I can set different 
            // microstepping without jumpers, via the main setup function.
            #define MS1_PIN 22
            #define MS2_PIN 24
            #define MS3_PIN 23
            pinMode( MS1_PIN,OUTPUT);
            pinMode( MS2_PIN,OUTPUT);
            pinMode( MS3_PIN,OUTPUT);
            digitalWrite(MS1_PIN,HIGH);
            digitalWrite(MS2_PIN,HIGH);
            digitalWrite(MS3_PIN,HIGH);
            
            // Setup the LCD screen here
            lcd.print("Initializing...");
          
        }
        // custom loop function code 
        void loopLcdUpdate()
        {
            // all global variable defined in Sprinter.pde must be declared as extern in here, or else
            // the header fails to compile. 
            // don't worry as this function will ONLY be called after tt has already being initialized!
            extern int tt; 
            
            // prints the extruder temperature on an LCD screen. 
            String temp = "Temp: ";
            temp = temp+String(tt);
            lcd.print(temp);
        }
    #endif
    
    //#define CUSTOM_CODES
    #ifdef CUSTOM_CODES
        #define CUSTOM_M_CODES \
            case 300:\
                Serial.print("custom M code 300 works!\n");\
                break;
                
        #define CUSTOM_G_CODES \
            case 300:\
                Serial.print("custom G code 300 works!\n");\
                break;
    #endif
    
    //=========================================================
#endif
