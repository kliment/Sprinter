// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#include "Tonokip_Firmware.h"
#include "configuration.h"
#include "pins.h"

#ifdef SDSUPPORT
#include "SdFat.h"
#endif

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G4  - Dwell S<seconds> or P<milliseconds>
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Wait for extruder current temp to reach target temp.
// M114 - Display current position

//Custom M Codes
// M80  - Turn on Power Supply
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move, 
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M115	- Capabilities string
// M140 - Set bed target temp
// M190 - Wait for bed current temp to reach target temp.
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000)


//Stepper Movement Variables
char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
bool move_direction[NUM_AXIS];
const int STEP_PIN[NUM_AXIS] = {X_STEP_PIN, Y_STEP_PIN, Z_STEP_PIN, E_STEP_PIN};
unsigned long axis_previous_micros[NUM_AXIS];
unsigned long previous_micros = 0, previous_millis_heater, previous_millis_bed_heater;
unsigned long move_steps_to_take[NUM_AXIS];
#ifdef RAMP_ACCELERATION
  unsigned long axis_max_interval[] = {100000000.0 / (max_start_speed_units_per_second[0] * axis_steps_per_unit[0]),
      100000000.0 / (max_start_speed_units_per_second[1] * axis_steps_per_unit[1]),
      100000000.0 / (max_start_speed_units_per_second[2] * axis_steps_per_unit[2]),
      100000000.0 / (max_start_speed_units_per_second[3] * axis_steps_per_unit[3])}; //TODO: refactor all things like this in a function, or move to setup()
                                                                                     // in a for loop
  unsigned long max_interval;
  unsigned long axis_steps_per_sqr_second[] = {max_acceleration_units_per_sq_second[0] * axis_steps_per_unit[0],
        max_acceleration_units_per_sq_second[1] * axis_steps_per_unit[1], max_acceleration_units_per_sq_second[2] * axis_steps_per_unit[2],
        max_acceleration_units_per_sq_second[3] * axis_steps_per_unit[3]};
  unsigned long axis_travel_steps_per_sqr_second[] = {max_travel_acceleration_units_per_sq_second[0] * axis_steps_per_unit[0],
        max_travel_acceleration_units_per_sq_second[1] * axis_steps_per_unit[1], max_travel_acceleration_units_per_sq_second[2] * axis_steps_per_unit[2],
        max_travel_acceleration_units_per_sq_second[3] * axis_steps_per_unit[3]};
  unsigned long steps_per_sqr_second, plateau_steps;
#endif
boolean acceleration_enabled = false, accelerating = false;
unsigned long interval;
float destination[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
float current_position[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
long axis_interval[NUM_AXIS]; // for speed delay
bool home_all_axis = true;
float feedrate = 1500, next_feedrate, saved_feedrate;
float time_for_move;
long gcode_N, gcode_LastN;
bool relative_mode = false;  //Determines Absolute or Relative Coordinates
bool relative_mode_e = false;  //Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.
long timediff = 0;
//experimental feedrate calc
float d = 0;
float axis_diff[NUM_AXIS] = {0, 0, 0, 0};
#ifdef STEP_DELAY_RATIO
  long long_step_delay_ratio = STEP_DELAY_RATIO * 100;
#endif


// comm variables
#define MAX_CMD_SIZE 96
#define BUFSIZE 8
char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
bool fromsd[BUFSIZE];
int bufindr = 0;
int bufindw = 0;
int buflen = 0;
int i = 0;
char serial_char;
int serial_count = 0;
boolean comment_mode = false;
char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

// Manage heater variables. For a thermistor or AD595 thermocouple, raw values refer to the 
// reading from the analog pin. For a MAX6675 thermocouple, the raw value is the temperature in 0.25 
// degree increments (i.e. 100=25 deg). 

int target_raw = 0;
int current_raw = 0;
int target_bed_raw = 0;
int current_bed_raw = 0;
float tt = 0, bt = 0;
#ifdef PIDTEMP
  int temp_iState = 0;
  int temp_dState = 0;
  int pTerm;
  int iTerm;
  int dTerm;
      //int output;
  int error;
  int temp_iState_min = 100 * -PID_INTEGRAL_DRIVE_MAX / PID_IGAIN;
  int temp_iState_max = 100 * PID_INTEGRAL_DRIVE_MAX / PID_IGAIN;
#endif
#ifdef SMOOTHING
  uint32_t nma = SMOOTHFACTOR * analogRead(TEMP_0_PIN);
#endif
#ifdef WATCHPERIOD
  int watch_raw = -1000;
  unsigned long watchmillis = 0;
#endif
#ifdef MINTEMP
  int minttemp = temp2analog(MINTEMP);
#endif
#ifdef MAXTEMP
int maxttemp = temp2analog(MAXTEMP);
#endif
        
//Inactivity shutdown variables
unsigned long previous_millis_cmd = 0;
unsigned long max_inactive_time = 0;
unsigned long stepper_inactive_time = 0;

#ifdef SDSUPPORT
  Sd2Card card;
  SdVolume volume;
  SdFile root;
  SdFile file;
  uint32_t filesize = 0;
  uint32_t sdpos = 0;
  bool sdmode = false;
  bool sdactive = false;
  bool savetosd = false;
  int16_t n;
  
  void initsd(){
  sdactive = false;
  #if SDSS >- 1
    if(root.isOpen())
        root.close();
    if (!card.init(SPI_FULL_SPEED,SDSS)){
        //if (!card.init(SPI_HALF_SPEED,SDSS))
          Serial.println("SD init fail");
    }
    else if (!volume.init(&card))
          Serial.println("volume.init failed");
    else if (!root.openRoot(&volume)) 
          Serial.println("openRoot failed");
    else 
            sdactive = true;
  #endif
  }
  
  inline void write_command(char *buf){
      char* begin = buf;
      char* npos = 0;
      char* end = buf + strlen(buf) - 1;
      
      file.writeError = false;
      if((npos = strchr(buf, 'N')) != NULL){
          begin = strchr(npos, ' ') + 1;
          end = strchr(npos, '*') - 1;
      }
      end[1] = '\r';
      end[2] = '\n';
      end[3] = '\0';
      //Serial.println(begin);
      file.write(begin);
      if (file.writeError){
          Serial.println("error writing to file");
      }
  }
#endif


void setup()
{ 
  Serial.begin(BAUDRATE);
  Serial.println("start");
  for(int i = 0; i < BUFSIZE; i++){
      fromsd[i] = false;
  }

  //Initialize Step Pins
  for(int i=0; i < NUM_AXIS; i++) if(STEP_PIN[i] > -1) pinMode(STEP_PIN[i],OUTPUT);
  
  //Initialize Dir Pins
  if(X_DIR_PIN > -1) pinMode(X_DIR_PIN,OUTPUT);
  if(Y_DIR_PIN > -1) pinMode(Y_DIR_PIN,OUTPUT);
  if(Z_DIR_PIN > -1) pinMode(Z_DIR_PIN,OUTPUT);
  if(E_DIR_PIN > -1) pinMode(E_DIR_PIN,OUTPUT);

  //Steppers default to disabled.
  if(X_ENABLE_PIN > -1) if(!X_ENABLE_ON) digitalWrite(X_ENABLE_PIN,HIGH);
  if(Y_ENABLE_PIN > -1) if(!Y_ENABLE_ON) digitalWrite(Y_ENABLE_PIN,HIGH);
  if(Z_ENABLE_PIN > -1) if(!Z_ENABLE_ON) digitalWrite(Z_ENABLE_PIN,HIGH);
  if(E_ENABLE_PIN > -1) if(!E_ENABLE_ON) digitalWrite(E_ENABLE_PIN,HIGH);

  //endstop pullups
  #ifdef ENDSTOPPULLUPS
    if(X_MIN_PIN > -1) { pinMode(X_MIN_PIN,INPUT); digitalWrite(X_MIN_PIN,HIGH);}
    if(Y_MIN_PIN > -1) { pinMode(Y_MIN_PIN,INPUT); digitalWrite(Y_MIN_PIN,HIGH);}
    if(Z_MIN_PIN > -1) { pinMode(Z_MIN_PIN,INPUT); digitalWrite(Z_MIN_PIN,HIGH);}
    if(X_MAX_PIN > -1) { pinMode(X_MAX_PIN,INPUT); digitalWrite(X_MAX_PIN,HIGH);}
    if(Y_MAX_PIN > -1) { pinMode(Y_MAX_PIN,INPUT); digitalWrite(Y_MAX_PIN,HIGH);}
    if(Z_MAX_PIN > -1) { pinMode(Z_MAX_PIN,INPUT); digitalWrite(Z_MAX_PIN,HIGH);}
  #endif
  //Initialize Enable Pins
  if(X_ENABLE_PIN > -1) pinMode(X_ENABLE_PIN,OUTPUT);
  if(Y_ENABLE_PIN > -1) pinMode(Y_ENABLE_PIN,OUTPUT);
  if(Z_ENABLE_PIN > -1) pinMode(Z_ENABLE_PIN,OUTPUT);
  if(E_ENABLE_PIN > -1) pinMode(E_ENABLE_PIN,OUTPUT);

  if(HEATER_0_PIN > -1) pinMode(HEATER_0_PIN,OUTPUT);
  if(HEATER_1_PIN > -1) pinMode(HEATER_1_PIN,OUTPUT);
  
#ifdef HEATER_USES_MAX6675
  digitalWrite(SCK_PIN,0);
  pinMode(SCK_PIN,OUTPUT);

  digitalWrite(MOSI_PIN,1);
  pinMode(MOSI_PIN,OUTPUT);

  digitalWrite(MISO_PIN,1);
  pinMode(MISO_PIN,INPUT);

  digitalWrite(MAX6675_SS,1);
  pinMode(MAX6675_SS,OUTPUT);
#endif  
 
#ifdef SDSUPPORT

  //power to SD reader
  #if SDPOWER > -1
    pinMode(SDPOWER,OUTPUT); 
    digitalWrite(SDPOWER,HIGH);
  #endif
  initsd();

#endif

}


void loop()
{
  if(buflen<3)
	get_command();
  
  if(buflen){
#ifdef SDSUPPORT
    if(savetosd){
        if(strstr(cmdbuffer[bufindr],"M29") == NULL){
            write_command(cmdbuffer[bufindr]);
            Serial.println("ok");
        }else{
            file.sync();
            file.close();
            savetosd = false;
            Serial.println("Done saving file.");
        }
    }else{
        process_commands();
    }
#else
    process_commands();
#endif
    buflen = (buflen-1);
    bufindr = (bufindr + 1)%BUFSIZE;
    }
  //check heater every n milliseconds
      manage_heater();
      manage_inactivity(1);
  }


inline void get_command() 
{ 
  while( Serial.available() > 0  && buflen < BUFSIZE) {
    serial_char = Serial.read();
    if(serial_char == '\n' || serial_char == '\r' || serial_char == ':' || serial_count >= (MAX_CMD_SIZE - 1) ) 
    {
      if(!serial_count) return; //if empty line
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if(!comment_mode){
    fromsd[bufindw] = false;
  if(strstr(cmdbuffer[bufindw], "N") != NULL)
  {
    strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
    gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
    if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], "M110") == NULL) ) {
      Serial.print("Serial Error: Line Number is not Last Line Number+1, Last Line:");
      Serial.println(gcode_LastN);
      //Serial.println(gcode_N);
      FlushSerialRequestResend();
      serial_count = 0;
      return;
    }
    
    if(strstr(cmdbuffer[bufindw], "*") != NULL)
    {
      byte checksum = 0;
      byte count = 0;
      while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
      strchr_pointer = strchr(cmdbuffer[bufindw], '*');
  
      if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
        Serial.print("Error: checksum mismatch, Last Line:");
        Serial.println(gcode_LastN);
        FlushSerialRequestResend();
        serial_count = 0;
        return;
      }
      //if no errors, continue parsing
    }
    else 
    {
      Serial.print("Error: No Checksum with line number, Last Line:");
      Serial.println(gcode_LastN);
      FlushSerialRequestResend();
      serial_count = 0;
      return;
    }
    
    gcode_LastN = gcode_N;
    //if no errors, continue parsing
  }
  else  // if we don't receive 'N' but still see '*'
  {
    if((strstr(cmdbuffer[bufindw], "*") != NULL))
    {
      Serial.print("Error: No Line Number with checksum, Last Line:");
      Serial.println(gcode_LastN);
      serial_count = 0;
      return;
    }
  }
	if((strstr(cmdbuffer[bufindw], "G") != NULL)){
		strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
		switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
		case 0:
		case 1:
              #ifdef SDSUPPORT
              if(savetosd)
                break;
              #endif
			  Serial.println("ok"); 
			  break;
		default:
			break;
		}

	}
        bufindw = (bufindw + 1)%BUFSIZE;
        buflen += 1;
        
      }
      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
#ifdef SDSUPPORT
if(!sdmode || serial_count!=0){
    return;
}
  while( filesize > sdpos  && buflen < BUFSIZE) {
    n = file.read();
    serial_char = (char)n;
    if(serial_char == '\n' || serial_char == '\r' || serial_char == ':' || serial_count >= (MAX_CMD_SIZE - 1) || n == -1) 
    {
        sdpos = file.curPosition();
        if(sdpos >= filesize){
            sdmode = false;
            Serial.println("Done printing file");
        }
      if(!serial_count) return; //if empty line
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if(!comment_mode){
        fromsd[bufindw] = true;
        buflen += 1;
        bufindw = (bufindw + 1)%BUFSIZE;
      }
      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
}
#endif

}


inline float code_value() { return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL)); }
inline long code_value_long() { return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10)); }
inline bool code_seen(char code_string[]) { return (strstr(cmdbuffer[bufindr], code_string) != NULL); }  //Return True if the string was found

inline bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

inline void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;

  if(code_seen('G'))
  {
    switch((int)code_value())
    {
      case 0: // G0 -> G1
      case 1: // G1
        #ifdef DISABLE_CHECK_DURING_ACC || DISABLE_CHECK_DURING_MOVE || DISABLE_CHECK_DURING_TRAVEL
          manage_heater();
        #endif
        get_coordinates(); // For X Y Z E F
        prepare_move();
        previous_millis_cmd = millis();
        //ClearToSend();
        return;
        //break;
      case 4: // G4 dwell
        codenum = 0;
        if(code_seen('P')) codenum = code_value(); // milliseconds to wait
        if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
        codenum += millis();  // keep track of when we started waiting
        while(millis()  < codenum ){
          manage_heater();
        }
        break;
      case 28: //G28 Home all Axis one at a time
        saved_feedrate = feedrate;
        for(int i=0; i < NUM_AXIS; i++) {
          destination[i] = 0;
          current_position[i] = 0;
        }
        feedrate = 0;

        home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));

        if((home_all_axis) || (code_seen('X'))) {
          if((X_MIN_PIN > -1 && X_HOME_DIR==-1) || (X_MAX_PIN > -1 && X_HOME_DIR==1)) {
            current_position[0] = 0;
            destination[0] = 1.5 * X_MAX_LENGTH * X_HOME_DIR;
            feedrate = max_start_speed_units_per_second[0] * 60;
            prepare_move();
          
            current_position[0] = 0;
            destination[0] = -1 * X_HOME_DIR;
            prepare_move();
          
            destination[0] = 10 * X_HOME_DIR;
            prepare_move();
          
            current_position[0] = 0;
            destination[0] = 0;
            feedrate = 0;
          }
        }
        
        if((home_all_axis) || (code_seen('X'))) {
          if((Y_MIN_PIN > -1 && Y_HOME_DIR==-1) || (Y_MAX_PIN > -1 && Y_HOME_DIR==1)) {
            current_position[1] = 0;
            destination[1] = 1.5 * Y_MAX_LENGTH * Y_HOME_DIR;
            feedrate = max_start_speed_units_per_second[1] * 60;
            prepare_move();
          
            current_position[1] = 0;
            destination[1] = -1 * Y_HOME_DIR;
            prepare_move();
          
            destination[1] = 10 * Y_HOME_DIR;
            prepare_move();
          
            current_position[1] = 0;
            destination[1] = 0;
            feedrate = 0;
          }
        }
        
        if((home_all_axis) || (code_seen('X'))) {
          if((Z_MIN_PIN > -1 && Z_HOME_DIR==-1) || (Z_MAX_PIN > -1 && Z_HOME_DIR==1)) {
            current_position[2] = 0;
            destination[2] = 1.5 * Z_MAX_LENGTH * Z_HOME_DIR;
            feedrate = max_feedrate[2]/2;
            prepare_move();
          
            current_position[2] = 0;
            destination[2] = -1 * Z_HOME_DIR;
            prepare_move();
          
            destination[2] = 10 * Z_HOME_DIR;
            prepare_move();
          
            current_position[2] = 0;
            destination[2] = 0;
            feedrate = 0;
          }
        }
        
        feedrate = saved_feedrate;
        previous_millis_cmd = millis();
        break;
      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;
      case 92: // G92
        for(int i=0; i < NUM_AXIS; i++) {
          if(code_seen(axis_codes[i])) current_position[i] = code_value();  
        }
        break;
        
    }
  }

  else if(code_seen('M'))
  {
    
    switch( (int)code_value() ) 
    {
#ifdef SDSUPPORT
        
      case 20: // M20 - list SD card
        Serial.println("Begin file list");
        root.ls();
        Serial.println("End file list");
        break;
      case 21: // M21 - init SD card
        sdmode = false;
        initsd();
        break;
      case 22: //M22 - release SD card
        sdmode = false;
        sdactive = false;
        break;
      case 23: //M23 - Select file
        if(sdactive){
            sdmode = false;
            file.close();
            starpos = (strchr(strchr_pointer + 4,'*'));
            if(starpos!=NULL)
                *(starpos-1)='\0';
            if (file.open(&root, strchr_pointer + 4, O_READ)) {
                Serial.print("File opened:");
                Serial.print(strchr_pointer + 4);
                Serial.print(" Size:");
                Serial.println(file.fileSize());
                sdpos = 0;
                filesize = file.fileSize();
                Serial.println("File selected");
            }
            else{
                Serial.println("file.open failed");
            }
        }
        break;
      case 24: //M24 - Start SD print
        if(sdactive){
            sdmode = true;
        }
        break;
      case 25: //M25 - Pause SD print
        if(sdmode){
            sdmode = false;
        }
        break;
      case 26: //M26 - Set SD index
        if(sdactive && code_seen('S')){
            sdpos = code_value_long();
            file.seekSet(sdpos);
        }
        break;
      case 27: //M27 - Get SD status
        if(sdactive){
            Serial.print("SD printing byte ");
            Serial.print(sdpos);
            Serial.print("/");
            Serial.println(filesize);
        }else{
            Serial.println("Not SD printing");
        }
        break;
            case 28: //M28 - Start SD write
        if(sdactive){
          char* npos = 0;
            file.close();
            sdmode = false;
            starpos = (strchr(strchr_pointer + 4,'*'));
            if(starpos != NULL){
              npos = strchr(cmdbuffer[bufindr], 'N');
              strchr_pointer = strchr(npos,' ') + 1;
              *(starpos-1) = '\0';
            }
      if (!file.open(&root, strchr_pointer+4, O_CREAT | O_APPEND | O_WRITE | O_TRUNC))
            {
            Serial.print("open failed, File: ");
            Serial.print(strchr_pointer + 4);
            Serial.print(".");
            }else{
            savetosd = true;
            Serial.print("Writing to file: ");
            Serial.println(strchr_pointer + 4);
            }
        }
        break;
      case 29: //M29 - Stop SD write
        //processed in write to file routine above
        //savetosd = false;
        break;
#endif
      case 104: // M104
        if (code_seen('S')) target_raw = temp2analog(code_value());
        #ifdef WATCHPERIOD
            if(target_raw > current_raw){
                watchmillis = max(1,millis());
                watch_raw = current_raw;
            }else{
                watchmillis = 0;
            }
        #endif
        break;
      case 140: // M140 set bed temp
        if (code_seen('S')) target_bed_raw = temp2analogBed(code_value());
        break;
      case 105: // M105
        #if (TEMP_0_PIN > -1) || defined (HEATER_USES_MAX6675)
          tt = analog2temp(current_raw);
        #endif
        #if TEMP_1_PIN > -1
          bt = analog2tempBed(current_bed_raw);
        #endif
        #if (TEMP_0_PIN > -1) || defined (HEATER_USES_MAX6675)
            Serial.print("ok T:");
            Serial.print(tt); 
          #if TEMP_1_PIN > -1
            Serial.print(" B:");
            Serial.println(bt); 
          #else
            Serial.println();
          #endif
        #else
          Serial.println("No thermistors - no temp");
        #endif
        return;
        //break;
      case 109: // M109 - Wait for extruder heater to reach target.
        if (code_seen('S')) target_raw = temp2analog(code_value());
        #ifdef WATCHPERIOD
            if(target_raw>current_raw){
                watchmillis = max(1,millis());
                watch_raw = current_raw;
            }else{
                watchmillis = 0;
            }
        #endif
        codenum = millis(); 
        while(current_raw < target_raw) {
          if( (millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            Serial.print("T:");
            Serial.println( analog2temp(current_raw) ); 
            codenum = millis(); 
          }
          manage_heater();
        }
        break;
      case 190: // M190 - Wait bed for heater to reach target.
      #if TEMP_1_PIN > -1
        if (code_seen('S')) target_bed_raw = temp2analog(code_value());
        codenum = millis(); 
        while(current_bed_raw < target_bed_raw) {
          if( (millis()-codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            tt=analog2temp(current_raw);
            Serial.print("T:");
            Serial.println( tt );
            Serial.print("ok T:");
            Serial.print( tt ); 
            Serial.print(" B:");
            Serial.println( analog2temp(current_bed_raw) ); 
            codenum = millis(); 
          }
            manage_heater();
        }
      #endif
      break;
      case 106: //M106 Fan On
        if (code_seen('S')){
            digitalWrite(FAN_PIN, HIGH);
            analogWrite(FAN_PIN, constrain(code_value(),0,255) );
        }
        else
            digitalWrite(FAN_PIN, HIGH);
        break;
      case 107: //M107 Fan Off
        analogWrite(FAN_PIN, 0);
        
        digitalWrite(FAN_PIN, LOW);
        break;
      case 80: // M81 - ATX Power On
        if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,OUTPUT); //GND
        break;
      case 81: // M81 - ATX Power Off
        if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT); //Floating
        break;
      case 82:
        axis_relative_modes[3] = false;
        break;
      case 83:
        axis_relative_modes[3] = true;
        break;
      case 84:
        if(code_seen('S')){ stepper_inactive_time = code_value() * 1000; }
        else{ disable_x(); disable_y(); disable_z(); disable_e(); }
        break;
      case 85: // M85
        code_seen('S');
        max_inactive_time = code_value() * 1000; 
        break;
      case 92: // M92
        for(int i=0; i < NUM_AXIS; i++) {
          if(code_seen(axis_codes[i])) axis_steps_per_unit[i] = code_value();
        }
        
        //Update start speed intervals and axis order. TODO: refactor axis_max_interval[] calculation into a function, as it
        // should also be used in setup() as well
        #ifdef RAMP_ACCELERATION
          long temp_max_intervals[NUM_AXIS];
          for(int i=0; i < NUM_AXIS; i++) {
            axis_max_interval[i] = 100000000.0 / (max_start_speed_units_per_second[i] * axis_steps_per_unit[i]);//TODO: do this for
                  // all steps_per_unit related variables
          }
        #endif
        break;
      case 115: // M115
        Serial.println("FIRMWARE_NAME:Sprinter FIRMWARE_URL:http%%3A/github.com/kliment/Sprinter/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1");
        break;
      case 114: // M114
	Serial.print("X:");
        Serial.print(current_position[0]);
	Serial.print("Y:");
        Serial.print(current_position[1]);
	Serial.print("Z:");
        Serial.print(current_position[2]);
	Serial.print("E:");
        Serial.println(current_position[3]);
        break;
      #ifdef RAMP_ACCELERATION
      //TODO: update for all axis, use for loop
      case 201: // M201
        for(int i=0; i < NUM_AXIS; i++) {
          if(code_seen(axis_codes[i])) axis_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
        }
        break;
      case 202: // M202
        for(int i=0; i < NUM_AXIS; i++) {
          if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
        }
        break;
      #endif
    }
    
  }
  else{
      Serial.println("Unknown command:");
      Serial.println(cmdbuffer[bufindr]);
  }
  
  ClearToSend();
      
}

inline void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  Serial.flush();
  Serial.print("Resend:");
  Serial.println(gcode_LastN + 1);
  ClearToSend();
}

inline void ClearToSend()
{
  previous_millis_cmd = millis();
  #ifdef SDSUPPORT
  if(fromsd[bufindr])
    return;
  #endif
  Serial.println("ok"); 
}

inline void get_coordinates()
{
  for(int i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i])) destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
    else destination[i] = current_position[i];                                                       //Are these else lines really needed?
  }
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

inline void prepare_move()
{
  //Find direction
  for(int i=0; i < NUM_AXIS; i++) {
    if(destination[i] >= current_position[i]) move_direction[i] = 1;
    else move_direction[i] = 0;
  }
  
  
  if (min_software_endstops) {
    if (destination[0] < 0) destination[0] = 0.0;
    if (destination[1] < 0) destination[1] = 0.0;
    if (destination[2] < 0) destination[2] = 0.0;
  }

  if (max_software_endstops) {
    if (destination[0] > X_MAX_LENGTH) destination[0] = X_MAX_LENGTH;
    if (destination[1] > Y_MAX_LENGTH) destination[1] = Y_MAX_LENGTH;
    if (destination[2] > Z_MAX_LENGTH) destination[2] = Z_MAX_LENGTH;
  }

  for(int i=0; i < NUM_AXIS; i++) {
    axis_diff[i] = destination[i] - current_position[i];
    move_steps_to_take[i] = abs(axis_diff[i]) * axis_steps_per_unit[i];
  }
  if(feedrate < 10)
      feedrate = 10;
  
  //Feedrate calc based on XYZ travel distance
  float xy_d;
  if(abs(axis_diff[0]) > 0 || abs(axis_diff[1]) > 0 || abs(axis_diff[2])) {
    xy_d = sqrt(axis_diff[0] * axis_diff[0] + axis_diff[1] * axis_diff[1]);
    d = sqrt(xy_d * xy_d + axis_diff[2] * axis_diff[2]);
  }
  else if(abs(axis_diff[3]) > 0)
    d = abs(axis_diff[3]);
  #ifdef DEBUG_PREPARE_MOVE
    else {
      log_message("_PREPARE_MOVE - No steps to take!");
    }
  #endif
  time_for_move = (d / (feedrate / 60000000.0) );
  //Check max feedrate for each axis is not violated, update time_for_move if necessary
  for(int i = 0; i < NUM_AXIS; i++) {
    if(move_steps_to_take[i] && abs(axis_diff[i]) / (time_for_move / 60000000.0) > max_feedrate[i]) {
      time_for_move = time_for_move / max_feedrate[i] * (abs(axis_diff[i]) / (time_for_move / 60000000.0));
    }
  }
  //Calculate the full speed stepper interval for each axis
  for(int i=0; i < NUM_AXIS; i++) {
    if(move_steps_to_take[i]) axis_interval[i] = time_for_move / move_steps_to_take[i] * 100;
  }
  
  #ifdef DEBUG_PREPARE_MOVE
    log_float("_PREPARE_MOVE - Move distance on the XY plane", xy_d);
    log_float("_PREPARE_MOVE - Move distance on the XYZ space", d);
    log_float("_PREPARE_MOVE - Commanded feedrate", feedrate);
    log_float("_PREPARE_MOVE - Constant full speed move time", time_for_move);
    log_float_array("_PREPARE_MOVE - Destination", destination, NUM_AXIS);
    log_float_array("_PREPARE_MOVE - Current position", current_position, NUM_AXIS);
    log_ulong_array("_PREPARE_MOVE - Steps to take", move_steps_to_take, NUM_AXIS);
    log_long_array("_PREPARE_MOVE - Axes full speed intervals", axis_interval, NUM_AXIS);
  #endif

  unsigned long move_steps[NUM_AXIS];
  for(int i=0; i < NUM_AXIS; i++)
    move_steps[i] = move_steps_to_take[i];
  linear_move(move_steps); // make the move
}

void linear_move(unsigned long axis_steps_remaining[]) // make linear move with preset speeds and destinations, see G0 and G1
{
  //Determine direction of movement
  if (destination[0] > current_position[0]) digitalWrite(X_DIR_PIN,!INVERT_X_DIR);
  else digitalWrite(X_DIR_PIN,INVERT_X_DIR);
  if (destination[1] > current_position[1]) digitalWrite(Y_DIR_PIN,!INVERT_Y_DIR);
  else digitalWrite(Y_DIR_PIN,INVERT_Y_DIR);
  if (destination[2] > current_position[2]) digitalWrite(Z_DIR_PIN,!INVERT_Z_DIR);
  else digitalWrite(Z_DIR_PIN,INVERT_Z_DIR);
  if (destination[3] > current_position[3]) digitalWrite(E_DIR_PIN,!INVERT_E_DIR);
  else digitalWrite(E_DIR_PIN,INVERT_E_DIR);
  
  if(X_MIN_PIN > -1) if(!move_direction[0]) if(digitalRead(X_MIN_PIN) != ENDSTOPS_INVERTING) axis_steps_remaining[0]=0;
  if(Y_MIN_PIN > -1) if(!move_direction[1]) if(digitalRead(Y_MIN_PIN) != ENDSTOPS_INVERTING) axis_steps_remaining[1]=0;
  if(Z_MIN_PIN > -1) if(!move_direction[2]) if(digitalRead(Z_MIN_PIN) != ENDSTOPS_INVERTING) axis_steps_remaining[2]=0;
  if(X_MAX_PIN > -1) if(move_direction[0]) if(digitalRead(X_MAX_PIN) != ENDSTOPS_INVERTING) axis_steps_remaining[0]=0;
  if(Y_MAX_PIN > -1) if(move_direction[1]) if(digitalRead(Y_MAX_PIN) != ENDSTOPS_INVERTING) axis_steps_remaining[1]=0;
  if(Z_MAX_PIN > -1) if(move_direction[2]) if(digitalRead(Z_MAX_PIN) != ENDSTOPS_INVERTING) axis_steps_remaining[2]=0;
  
  
  //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
  // TODO: maybe it's better to refactor into a generic enable(int axis) function, that will probably take more ram,
  // but will reduce code size
  if(axis_steps_remaining[0]) enable_x();
  if(axis_steps_remaining[1]) enable_y();
  if(axis_steps_remaining[2]) enable_z();
  if(axis_steps_remaining[3]) enable_e();

    //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
  unsigned long delta[] = {axis_steps_remaining[0], axis_steps_remaining[1], axis_steps_remaining[2], axis_steps_remaining[3]}; //TODO: implement a "for" to support N axes
  long axis_error[NUM_AXIS];
  unsigned int primary_axis;
  if(delta[1] > delta[0] && delta[1] > delta[2] && delta[1] > delta[3]) primary_axis = 1;
  else if (delta[0] >= delta[1] && delta[0] > delta[2] && delta[0] > delta[3]) primary_axis = 0;
  else if (delta[2] >= delta[0] && delta[2] >= delta[1] && delta[2] > delta[3]) primary_axis = 2;
  else primary_axis = 3;
  unsigned long steps_remaining = delta[primary_axis];
  unsigned long steps_to_take = steps_remaining;
  for(int i=0; i < NUM_AXIS; i++) if(i != primary_axis) axis_error[i] = delta[primary_axis] / 2;
  interval = axis_interval[primary_axis];
  bool is_print_move = delta[3] > 0;
  #ifdef DEBUG_BRESENHAM
    log_int("_BRESENHAM - Primary axis", primary_axis);
    log_int("_BRESENHAM - Primary axis full speed interval", interval);
    log_ulong_array("_BRESENHAM - Deltas", delta, NUM_AXIS);
    log_long_array("_BRESENHAM - Errors", axis_error, NUM_AXIS);
  #endif

  //If acceleration is enabled, do some Bresenham calculations depending on which axis will lead it.
  #ifdef RAMP_ACCELERATION
    long max_speed_steps_per_second;
    long min_speed_steps_per_second;
    max_interval = axis_max_interval[primary_axis];
    #ifdef DEBUG_RAMP_ACCELERATION
     log_ulong_array("_RAMP_ACCELERATION - Teoric step intervals at move start", axis_max_interval, NUM_AXIS);
    #endif
    unsigned long new_axis_max_intervals[NUM_AXIS];
    max_speed_steps_per_second = 100000000 / interval;
    min_speed_steps_per_second = 100000000 / max_interval; //TODO: can this be deleted?
    //Calculate start speeds based on moving axes max start speed constraints.
    int slowest_start_axis = primary_axis;
    unsigned long slowest_start_axis_max_interval = max_interval;
    for(int i = 0; i < NUM_AXIS; i++)
      if (axis_steps_remaining[i] >0 && i != primary_axis && axis_max_interval[i] * axis_steps_remaining[i]
              / axis_steps_remaining[slowest_start_axis] > slowest_start_axis_max_interval) {
        slowest_start_axis = i;
        slowest_start_axis_max_interval = axis_max_interval[i];
      }
    for(int i = 0; i < NUM_AXIS; i++)
      if(axis_steps_remaining[i] >0) {
        new_axis_max_intervals[i] = slowest_start_axis_max_interval * axis_steps_remaining[slowest_start_axis] / axis_steps_remaining[i];
        if(i == primary_axis) {
          max_interval = new_axis_max_intervals[i];
          min_speed_steps_per_second = 100000000 / max_interval;
        }
      }
    //Calculate slowest axis plateau time
    float slowest_axis_plateau_time = 0;
    for(int i=0; i < NUM_AXIS ; i++) {
      if(axis_steps_remaining[i] > 0) {
        if(is_print_move && axis_steps_remaining[i] > 0) slowest_axis_plateau_time = max(slowest_axis_plateau_time,
              (100000000.0 / axis_interval[i] - 100000000.0 / new_axis_max_intervals[i]) / (float) axis_steps_per_sqr_second[i]);
        else if(axis_steps_remaining[i] > 0) slowest_axis_plateau_time = max(slowest_axis_plateau_time,
              (100000000.0 / axis_interval[i] - 100000000.0 / new_axis_max_intervals[i]) / (float) axis_travel_steps_per_sqr_second[i]);
      }
    }
    //Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
    steps_per_sqr_second = (100000000.0 / axis_interval[primary_axis] - 100000000.0 / new_axis_max_intervals[primary_axis]) / slowest_axis_plateau_time;
    plateau_steps = (long) ((steps_per_sqr_second / 2.0 * slowest_axis_plateau_time + min_speed_steps_per_second) * slowest_axis_plateau_time);
    #ifdef DEBUG_RAMP_ACCELERATION
      log_int("_RAMP_ACCELERATION - Start speed limiting axis", slowest_start_axis);
      log_ulong("_RAMP_ACCELERATION - Limiting axis start interval", slowest_start_axis_max_interval);
      log_ulong_array("_RAMP_ACCELERATION - Actual step intervals at move start", new_axis_max_intervals, NUM_AXIS);
    #endif
  #endif
  
  unsigned long steps_done = 0;
  #ifdef RAMP_ACCELERATION
  plateau_steps *= 1.01; // This is to compensate we use discrete intervals
  acceleration_enabled = true;
  long full_interval = interval;
  if(interval > max_interval) acceleration_enabled = false;
  boolean decelerating = false;
  #endif
  
  unsigned long start_move_micros = micros();
  for(int i = 0; i < NUM_AXIS; i++) {
    axis_previous_micros[i] = start_move_micros * 100;
  }

  #ifdef DISABLE_CHECK_DURING_TRAVEL
    //If the move time is more than allowed in DISABLE_CHECK_DURING_TRAVEL, let's
    // consider this a print move and perform heat management during it
    if(time_for_move / 1000 > DISABLE_CHECK_DURING_TRAVEL) is_print_move = true;
    //else, if the move is a retract, consider it as a travel move for the sake of this feature
    else if(delta[3]>0 && delta[0] + delta[1] + delta[2] == 0) is_print_move = false;
    #ifdef DEBUG_DISABLE_CHECK_DURING_TRAVEL
      log_bool("_DISABLE_CHECK_DURING_TRAVEL - is_print_move", is_print_move);
    #endif
  #endif

  #ifdef DEBUG_MOVE_TIME
    unsigned long startmove = micros();
  #endif
  
  //move until no more steps remain 
  while(axis_steps_remaining[0] + axis_steps_remaining[1] + axis_steps_remaining[2] + axis_steps_remaining[3] > 0) {
    #ifdef DISABLE_CHECK_DURING_ACC
      if(!accelerating && !decelerating) {
        //If more that HEATER_CHECK_INTERVAL ms have passed since previous heating check, adjust temp
        #ifdef DISABLE_CHECK_DURING_TRAVEL
          if(is_print_move)
        #endif
            manage_heater();
      }
    #else
      #ifdef DISABLE_CHECK_DURING_MOVE
        {} //Do nothing
      #else
        //If more that HEATER_CHECK_INTERVAL ms have passed since previous heating check, adjust temp
        #ifdef DISABLE_CHECK_DURING_TRAVEL
          if(is_print_move)
        #endif
            manage_heater();
      #endif
    #endif
    #ifdef RAMP_ACCELERATION
    //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
    if (acceleration_enabled && steps_done == 0) {
        interval = max_interval;
    } else if (acceleration_enabled && steps_done <= plateau_steps) {
        long current_speed = (long) ((((long) steps_per_sqr_second) / 10000)
	    * ((micros() - start_move_micros)  / 100) + (long) min_speed_steps_per_second);
	    interval = 100000000 / current_speed;
      if (interval < full_interval) {
        accelerating = false;
      	interval = full_interval;
      }
      if (steps_done >= steps_to_take / 2) {
	plateau_steps = steps_done;
	max_speed_steps_per_second = 100000000 / interval;
	accelerating = false;
      }
    } else if (acceleration_enabled && steps_remaining <= plateau_steps) { //(interval > minInterval * 100) {
      if (!accelerating) {
        start_move_micros = micros();
        accelerating = true;
        decelerating = true;
      }				
      long current_speed = (long) ((long) max_speed_steps_per_second - ((((long) steps_per_sqr_second) / 10000)
          * ((micros() - start_move_micros) / 100)));
      interval = 100000000 / current_speed;
      if (interval > max_interval)
	interval = max_interval;
    } else {
      //Else, we are just use the full speed interval as current interval
      interval = full_interval;
      accelerating = false;
    }
    #endif

    //If there are x or y steps remaining, perform Bresenham algorithm
    if(axis_steps_remaining[primary_axis]) {
      if(X_MIN_PIN > -1) if(!move_direction[0]) if(digitalRead(X_MIN_PIN) != ENDSTOPS_INVERTING) break;
      if(Y_MIN_PIN > -1) if(!move_direction[1]) if(digitalRead(Y_MIN_PIN) != ENDSTOPS_INVERTING) break;
      if(X_MAX_PIN > -1) if(move_direction[0]) if(digitalRead(X_MAX_PIN) != ENDSTOPS_INVERTING) break;
      if(Y_MAX_PIN > -1) if(move_direction[1]) if(digitalRead(Y_MAX_PIN) != ENDSTOPS_INVERTING) break;
      if(Z_MIN_PIN > -1) if(!move_direction[2]) if(digitalRead(Z_MIN_PIN) != ENDSTOPS_INVERTING) break;
      if(Z_MAX_PIN > -1) if(move_direction[2]) if(digitalRead(Z_MAX_PIN) != ENDSTOPS_INVERTING) break;
      timediff = micros() * 100 - axis_previous_micros[primary_axis];
      while(timediff >= interval && axis_steps_remaining[primary_axis] > 0) {
        steps_done++;
        steps_remaining--;
        axis_steps_remaining[primary_axis]--; timediff -= interval;
        do_step_update_micros(primary_axis);
        for(int i=0; i < NUM_AXIS; i++) if(i != primary_axis && axis_steps_remaining[i] > 0) {
          axis_error[i] = axis_error[i] - delta[i];
          if(axis_error[i] < 0) {
            do_step(i); axis_steps_remaining[i]--;
            axis_error[i] = axis_error[i] + delta[primary_axis];
          }
        }
        #ifdef STEP_DELAY_RATIO
        if(timediff >= interval) delayMicroseconds(long_step_delay_ratio * interval / 10000);
        #endif
        #ifdef STEP_DELAY_MICROS
        if(timediff >= interval) delayMicroseconds(STEP_DELAY_MICROS);
        #endif
      }
    }
  }
  #ifdef DEBUG_MOVE_TIME
    log_ulong("_MOVE_TIME - This move took", micros()-startmove);
  #endif
  
  if(DISABLE_X) disable_x();
  if(DISABLE_Y) disable_y();
  if(DISABLE_Z) disable_z();
  if(DISABLE_E) disable_e();
  
  // Update current position partly based on direction, we probably can combine this with the direction code above...
  for(int i=0; i < NUM_AXIS; i++) {
    if (destination[i] > current_position[i]) current_position[i] = current_position[i] + move_steps_to_take[i] /  axis_steps_per_unit[i];
    else current_position[i] = current_position[i] - move_steps_to_take[i] / axis_steps_per_unit[i];
  }
}

inline void do_step_update_micros(int axis) {
  digitalWrite(STEP_PIN[axis], HIGH);
  axis_previous_micros[axis] += interval;
  digitalWrite(STEP_PIN[axis], LOW);
}

inline void do_step(int axis) {
  digitalWrite(STEP_PIN[axis], HIGH);
  digitalWrite(STEP_PIN[axis], LOW);
}

inline void disable_x() { if(X_ENABLE_PIN > -1) digitalWrite(X_ENABLE_PIN,!X_ENABLE_ON); }
inline void disable_y() { if(Y_ENABLE_PIN > -1) digitalWrite(Y_ENABLE_PIN,!Y_ENABLE_ON); }
inline void disable_z() { if(Z_ENABLE_PIN > -1) digitalWrite(Z_ENABLE_PIN,!Z_ENABLE_ON); }
inline void disable_e() { if(E_ENABLE_PIN > -1) digitalWrite(E_ENABLE_PIN,!E_ENABLE_ON); }
inline void  enable_x() { if(X_ENABLE_PIN > -1) digitalWrite(X_ENABLE_PIN, X_ENABLE_ON); }
inline void  enable_y() { if(Y_ENABLE_PIN > -1) digitalWrite(Y_ENABLE_PIN, Y_ENABLE_ON); }
inline void  enable_z() { if(Z_ENABLE_PIN > -1) digitalWrite(Z_ENABLE_PIN, Z_ENABLE_ON); }
inline void  enable_e() { if(E_ENABLE_PIN > -1) digitalWrite(E_ENABLE_PIN, E_ENABLE_ON); }

#define HEAT_INTERVAL 250
#ifdef HEATER_USES_MAX6675
unsigned long max6675_previous_millis = 0;
int max6675_temp = 2000;

inline int read_max6675()
{
  if (millis() - max6675_previous_millis < HEAT_INTERVAL) 
    return max6675_temp;
  
  max6675_previous_millis = millis();

  max6675_temp = 0;
    
  #ifdef	PRR
    PRR &= ~(1<<PRSPI);
  #elif defined PRR0
    PRR0 &= ~(1<<PRSPI);
  #endif
  
  SPCR = (1<<MSTR) | (1<<SPE) | (1<<SPR0);
  
  // enable TT_MAX6675
  digitalWrite(MAX6675_SS, 0);
  
  // ensure 100ns delay - a bit extra is fine
  delay(1);
  
  // read MSB
  SPDR = 0;
  for (;(SPSR & (1<<SPIF)) == 0;);
  max6675_temp = SPDR;
  max6675_temp <<= 8;
  
  // read LSB
  SPDR = 0;
  for (;(SPSR & (1<<SPIF)) == 0;);
  max6675_temp |= SPDR;
  
  // disable TT_MAX6675
  digitalWrite(MAX6675_SS, 1);

  if (max6675_temp & 4) 
  {
    // thermocouple open
    max6675_temp = 2000;
  }
  else 
  {
    max6675_temp = max6675_temp >> 3;
  }

  return max6675_temp;
}
#endif


inline void manage_heater()
{
  if((millis() - previous_millis_heater) < HEATER_CHECK_INTERVAL )
    return;
  previous_millis_heater = millis();
  #ifdef HEATER_USES_THERMISTOR
    current_raw = analogRead(TEMP_0_PIN); 
    #ifdef DEBUG_HEAT_MGMT
      log_int("_HEAT_MGMT - analogRead(TEMP_0_PIN)", current_raw);
      log_int("_HEAT_MGMT - NUMTEMPS", NUMTEMPS);
    #endif
    // When using thermistor, when the heater is colder than targer temp, we get a higher analog reading than target, 
    // this switches it up so that the reading appears lower than target for the control logic.
    current_raw = 1023 - current_raw;
  #elif defined HEATER_USES_AD595
    current_raw = analogRead(TEMP_0_PIN);    
  #elif defined HEATER_USES_MAX6675
    current_raw = read_max6675();
  #endif
  #ifdef SMOOTHING
  nma = (nma + current_raw) - (nma / SMOOTHFACTOR);
  current_raw = nma / SMOOTHFACTOR;
  #endif
  #ifdef WATCHPERIOD
    if(watchmillis && millis() - watchmillis > WATCHPERIOD){
        if(watch_raw + 1 >= current_raw){
            target_raw = 0;
            digitalWrite(HEATER_0_PIN,LOW);
            digitalWrite(LED_PIN,LOW);
        }else{
            watchmillis = 0;
        }
    }
  #endif
  #ifdef MINTEMP
    if(current_raw <= minttemp)
        target_raw = 0;
  #endif
  #ifdef MAXTEMP
    if(current_raw >= maxttemp) {
        target_raw = 0;
    }
  #endif
  #if (TEMP_0_PIN > -1) || defined (HEATER_USES_MAX66675)
    #ifdef PIDTEMP
      error = target_raw - current_raw;
      pTerm = (PID_PGAIN * error) / 100;
      temp_iState += error;
      temp_iState = constrain(temp_iState, temp_iState_min, temp_iState_max);
      iTerm = (PID_IGAIN * temp_iState) / 100;
      dTerm = (PID_DGAIN * (current_raw - temp_dState)) / 100;
      temp_dState = current_raw;
      analogWrite(HEATER_0_PIN, constrain(pTerm + iTerm - dTerm, 0, PID_MAX));
    #else
      if(current_raw >= target_raw)
      {
        digitalWrite(HEATER_0_PIN,LOW);
        digitalWrite(LED_PIN,LOW);
      }
      else 
      {
        digitalWrite(HEATER_0_PIN,HIGH);
        digitalWrite(LED_PIN,HIGH);
      }
    #endif
  #endif
    
  if(millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
    return;
  previous_millis_bed_heater = millis();
  
  #ifdef BED_USES_THERMISTOR
  
    current_bed_raw = analogRead(TEMP_1_PIN);   
    #ifdef DEBUG_HEAT_MGMT
      log_int("_HEAT_MGMT - analogRead(TEMP_1_PIN)", current_bed_raw);
      log_int("_HEAT_MGMT - BNUMTEMPS", BNUMTEMPS);
    #endif               
  
    // If using thermistor, when the heater is colder than targer temp, we get a higher analog reading than target, 
    // this switches it up so that the reading appears lower than target for the control logic.
    current_bed_raw = 1023 - current_bed_raw;
  #elif defined BED_USES_AD595
    current_bed_raw = analogRead(TEMP_1_PIN);                  

  #endif
  
  
  #if TEMP_1_PIN > -1
    if(current_bed_raw >= target_bed_raw)
    {
      digitalWrite(HEATER_1_PIN,LOW);
    }
    else 
    {
      digitalWrite(HEATER_1_PIN,HIGH);
    }
  #endif
}

// Takes hot end temperature value as input and returns corresponding raw value. 
// For a thermistor, it uses the RepRap thermistor temp table.
// This is needed because PID in hydra firmware hovers around a given analog value, not a temp value.
// This function is derived from inversing the logic from a portion of getTemperature() in FiveD RepRap firmware.
float temp2analog(int celsius) {
  #ifdef HEATER_USES_THERMISTOR
    int raw = 0;
    byte i;
    
    for (i=1; i<NUMTEMPS; i++)
    {
      if (temptable[i][1] < celsius)
      {
        raw = temptable[i-1][0] + 
          (celsius - temptable[i-1][1]) * 
          (temptable[i][0] - temptable[i-1][0]) /
          (temptable[i][1] - temptable[i-1][1]);
      
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == NUMTEMPS) raw = temptable[i-1][0];

    return 1023 - raw;
  #elif defined HEATER_USES_AD595
    return celsius * (1024.0 / (5.0 * 100.0) );
  #elif defined HEATER_USES_MAX6675
    return celsius * 4.0;
  #endif
}

// Takes bed temperature value as input and returns corresponding raw value. 
// For a thermistor, it uses the RepRap thermistor temp table.
// This is needed because PID in hydra firmware hovers around a given analog value, not a temp value.
// This function is derived from inversing the logic from a portion of getTemperature() in FiveD RepRap firmware.
float temp2analogBed(int celsius) {
  #ifdef BED_USES_THERMISTOR

    int raw = 0;
    byte i;
    
    for (i=1; i<BNUMTEMPS; i++)
    {
      if (bedtemptable[i][1] < celsius)
      {
        raw = bedtemptable[i-1][0] + 
          (celsius - bedtemptable[i-1][1]) * 
          (bedtemptable[i][0] - bedtemptable[i-1][0]) /
          (bedtemptable[i][1] - bedtemptable[i-1][1]);
      
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == BNUMTEMPS) raw = bedtemptable[i-1][0];

    return 1023 - raw;
  #elif defined BED_USES_AD595
    return celsius * (1024.0 / (5.0 * 100.0) );
  #endif
}

// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
float analog2temp(int raw) {
  #ifdef HEATER_USES_THERMISTOR
    int celsius = 0;
    byte i;
    
    raw = 1023 - raw;

    for (i=1; i<NUMTEMPS; i++)
    {
      if (temptable[i][0] > raw)
      {
        celsius  = temptable[i-1][1] + 
          (raw - temptable[i-1][0]) * 
          (temptable[i][1] - temptable[i-1][1]) /
          (temptable[i][0] - temptable[i-1][0]);

        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == NUMTEMPS) celsius = temptable[i-1][1];

    return celsius;
  #elif defined HEATER_USES_AD595
    return raw * ((5.0 * 100.0) / 1024.0);
  #elif defined HEATER_USES_MAX6675
    return raw * 0.25;
  #endif
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
float analog2tempBed(int raw) {
  #ifdef BED_USES_THERMISTOR
    int celsius = 0;
    byte i;

    raw = 1023 - raw;

    for (i=1; i<NUMTEMPS; i++)
    {
      if (bedtemptable[i][0] > raw)
      {
        celsius  = bedtemptable[i-1][1] + 
          (raw - bedtemptable[i-1][0]) * 
          (bedtemptable[i][1] - bedtemptable[i-1][1]) /
          (bedtemptable[i][0] - bedtemptable[i-1][0]);

        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == NUMTEMPS) celsius = bedtemptable[i-1][1];

    return celsius;
    
  #elif defined BED_USES_AD595
    return raw * ((5.0 * 100.0) / 1024.0);
  #endif
}

inline void kill()
{
  #if TEMP_0_PIN > -1
  target_raw=0;
  digitalWrite(HEATER_0_PIN,LOW);
  #endif
  #if TEMP_1_PIN > -1
  target_bed_raw=0;
  if(HEATER_1_PIN > -1) digitalWrite(HEATER_1_PIN,LOW);
  #endif
  disable_x();
  disable_y();
  disable_z();
  disable_e();
  
  if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT);
  
}

inline void manage_inactivity(byte debug) { 
if( (millis()-previous_millis_cmd) >  max_inactive_time ) if(max_inactive_time) kill(); 
if( (millis()-previous_millis_cmd) >  stepper_inactive_time ) if(stepper_inactive_time) { disable_x(); disable_y(); disable_z(); disable_e(); }
}

#ifdef DEBUG
void log_message(char*   message) {
  Serial.print("DEBUG"); Serial.println(message);
}

void log_bool(char* message, bool value) {
  Serial.print("DEBUG"); Serial.print(message); Serial.print(": "); Serial.println(value);
}

void log_int(char* message, int value) {
  Serial.print("DEBUG"); Serial.print(message); Serial.print(": "); Serial.println(value);
}

void log_long(char* message, long value) {
  Serial.print("DEBUG"); Serial.print(message); Serial.print(": "); Serial.println(value);
}

void log_float(char* message, float value) {
  Serial.print("DEBUG"); Serial.print(message); Serial.print(": "); Serial.println(value);
}

void log_uint(char* message, unsigned int value) {
  Serial.print("DEBUG"); Serial.print(message); Serial.print(": "); Serial.println(value);
}

void log_ulong(char* message, unsigned long value) {
  Serial.print("DEBUG"); Serial.print(message); Serial.print(": "); Serial.println(value);
}

void log_int_array(char* message, int value[], int array_lenght) {
  Serial.print("DEBUG"); Serial.print(message); Serial.print(": {");
  for(int i=0; i < array_lenght; i++){
    Serial.print(value[i]);
    if(i != array_lenght-1) Serial.print(", ");
  }
  Serial.println("}");
}

void log_long_array(char* message, long value[], int array_lenght) {
  Serial.print("DEBUG"); Serial.print(message); Serial.print(": {");
  for(int i=0; i < array_lenght; i++){
    Serial.print(value[i]);
    if(i != array_lenght-1) Serial.print(", ");
  }
  Serial.println("}");
}

void log_float_array(char* message, float value[], int array_lenght) {
  Serial.print("DEBUG"); Serial.print(message); Serial.print(": {");
  for(int i=0; i < array_lenght; i++){
    Serial.print(value[i]);
    if(i != array_lenght-1) Serial.print(", ");
  }
  Serial.println("}");
}

void log_uint_array(char* message, unsigned int value[], int array_lenght) {
  Serial.print("DEBUG"); Serial.print(message); Serial.print(": {");
  for(int i=0; i < array_lenght; i++){
    Serial.print(value[i]);
    if(i != array_lenght-1) Serial.print(", ");
  }
  Serial.println("}");
}

void log_ulong_array(char* message, unsigned long value[], int array_lenght) {
  Serial.print("DEBUG"); Serial.print(message); Serial.print(": {");
  for(int i=0; i < array_lenght; i++){
    Serial.print(value[i]);
    if(i != array_lenght-1) Serial.print(", ");
  }
  Serial.println("}");
}
#endif
