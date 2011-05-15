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
bool direction_x, direction_y, direction_z, direction_e;
unsigned long previous_micros = 0, previous_micros_x = 0, previous_micros_y = 0, previous_micros_z = 0, previous_micros_e = 0, previous_millis_heater, previous_millis_bed_heater;
unsigned long x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take;
#ifdef RAMP_ACCELERATION
  unsigned long max_x_interval = 100000000.0 / (min_units_per_second * x_steps_per_unit);
  unsigned long max_y_interval = 100000000.0 / (min_units_per_second * y_steps_per_unit);
  unsigned long max_interval;
  unsigned long x_steps_per_sqr_second = max_acceleration_units_per_sq_second * x_steps_per_unit;
  unsigned long y_steps_per_sqr_second = max_acceleration_units_per_sq_second * y_steps_per_unit;
  unsigned long x_travel_steps_per_sqr_second = max_travel_acceleration_units_per_sq_second * x_steps_per_unit;
  unsigned long y_travel_steps_per_sqr_second = max_travel_acceleration_units_per_sq_second * y_steps_per_unit;
  unsigned long steps_per_sqr_second, plateau_steps;
#endif
#ifdef EXP_ACCELERATION
  unsigned long long_full_velocity_units = full_velocity_units * 100;
  unsigned long long_travel_move_full_velocity_units = travel_move_full_velocity_units * 100;
  unsigned long max_x_interval = 100000000.0 / (min_units_per_second * x_steps_per_unit);
  unsigned long max_y_interval = 100000000.0 / (min_units_per_second * y_steps_per_unit);
  unsigned long max_interval;
  unsigned long x_min_constant_speed_steps = min_constant_speed_units * x_steps_per_unit,
    y_min_constant_speed_steps = min_constant_speed_units * y_steps_per_unit, min_constant_speed_steps;
#endif
boolean acceleration_enabled = false, accelerating = false;
unsigned long interval;
float destination_x = 0.0, destination_y = 0.0, destination_z = 0.0, destination_e = 0.0;
float current_x = 0.0, current_y = 0.0, current_z = 0.0, current_e = 0.0;
long x_interval, y_interval, z_interval, e_interval; // for speed delay
float feedrate = 1500, next_feedrate, z_feedrate, saved_feedrate;
float time_for_move;
long gcode_N, gcode_LastN;
bool relative_mode = false;  //Determines Absolute or Relative Coordinates
bool relative_mode_e = false;  //Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.
long timediff = 0;
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
  if(X_STEP_PIN > -1) pinMode(X_STEP_PIN,OUTPUT);
  if(Y_STEP_PIN > -1) pinMode(Y_STEP_PIN,OUTPUT);
  if(Z_STEP_PIN > -1) pinMode(Z_STEP_PIN,OUTPUT);
  if(E_STEP_PIN > -1) pinMode(E_STEP_PIN,OUTPUT);
  
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
 //experimental feedrate calc
float d = 0;
float xdiff = 0, ydiff = 0, zdiff = 0, ediff = 0;

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
        destination_x = 0;
        current_x = 0;
        destination_y = 0;
        current_y = 0;
        destination_z = 0;
        current_z = 0;
        destination_e = 0;
        current_e = 0;
        feedrate = 0;

    
        if((X_MIN_PIN > -1 && X_HOME_DIR==-1) || (X_MAX_PIN > -1 && X_HOME_DIR==1)) {
          current_x = 0;
          destination_x = 1.5 * X_MAX_LENGTH * X_HOME_DIR;
          feedrate = min_units_per_second * 60;
          prepare_move();
          
          current_x = 0;
          destination_x = -1 * X_HOME_DIR;
          prepare_move();
          
          destination_x = 10 * X_HOME_DIR;
          prepare_move();
          
          current_x = 0;
          destination_x = 0;
          feedrate = 0;
        }
        
        if((Y_MIN_PIN > -1 && Y_HOME_DIR==-1) || (Y_MAX_PIN > -1 && Y_HOME_DIR==1)) {
          current_y = 0;
          destination_y = 1.5 * Y_MAX_LENGTH * Y_HOME_DIR;
          feedrate = min_units_per_second * 60;
          prepare_move();
          
          current_y = 0;
          destination_y = -1 * Y_HOME_DIR;
          prepare_move();
          
          destination_y = 10 * Y_HOME_DIR;
          prepare_move();
          
          current_y = 0;
          destination_y = 0;
          feedrate = 0;
        }
        
        if((Z_MIN_PIN > -1 && Z_HOME_DIR==-1) || (Z_MAX_PIN > -1 && Z_HOME_DIR==1)) {
          current_z = 0;
          destination_z = 1.5 * Z_MAX_LENGTH * Z_HOME_DIR;
          feedrate = max_z_feedrate/2;
          prepare_move();
          
          current_z = 0;
          destination_z = -1 * Z_HOME_DIR;
          prepare_move();
          
          destination_z = 10 * Z_HOME_DIR;
          prepare_move();
          
          current_z = 0;
          destination_z = 0;
          feedrate = 0;
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
        if(code_seen('X')) current_x = code_value();
        if(code_seen('Y')) current_y = code_value();
        if(code_seen('Z')) current_z = code_value();
        if(code_seen('E')) current_e = code_value();
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
          Serial.print("T:");
          Serial.println(tt); 
          #if TEMP_1_PIN > -1
        
            Serial.print("ok T:");
            Serial.print(tt); 
            Serial.print(" B:");
            Serial.println(bt); 
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
        relative_mode_e = false;
        break;
      case 83:
        relative_mode_e = true;
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
        if(code_seen('X')) x_steps_per_unit = code_value();
        if(code_seen('Y')) y_steps_per_unit = code_value();
        if(code_seen('Z')) z_steps_per_unit = code_value();
        if(code_seen('E')) e_steps_per_unit = code_value();
        break;
      case 115: // M115
        Serial.println("FIRMWARE_NAME:Sprinter FIRMWARE_URL:http%%3A/github.com/kliment/Sprinter/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1");
        break;
      case 114: // M114
	Serial.print("X:");
        Serial.print(current_x);
	Serial.print("Y:");
        Serial.print(current_y);
	Serial.print("Z:");
        Serial.print(current_z);
	Serial.print("E:");
        Serial.println(current_e);
        break;
      #ifdef RAMP_ACCELERATION
      case 201: // M201
        if(code_seen('X')) x_steps_per_sqr_second = code_value() * x_steps_per_unit;
        if(code_seen('Y')) y_steps_per_sqr_second = code_value() * y_steps_per_unit;
        break;
      case 202: // M202
        if(code_seen('X')) x_travel_steps_per_sqr_second = code_value() * x_steps_per_unit;
        if(code_seen('Y')) y_travel_steps_per_sqr_second = code_value() * y_steps_per_unit;
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
  if(code_seen('X')) destination_x = (float)code_value() + relative_mode*current_x;
  else destination_x = current_x;                                                       //Are these else lines really needed?
  if(code_seen('Y')) destination_y = (float)code_value() + relative_mode*current_y;
  else destination_y = current_y;
  if(code_seen('Z')) destination_z = (float)code_value() + relative_mode*current_z;
  else destination_z = current_z;
  if(code_seen('E')) destination_e = (float)code_value() + (relative_mode_e || relative_mode)*current_e;
  else destination_e = current_e;
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

inline void prepare_move()
{
  //Find direction
  if(destination_x >= current_x) direction_x = 1;
  else direction_x = 0;
  if(destination_y >= current_y) direction_y = 1;
  else direction_y = 0;
  if(destination_z >= current_z) direction_z = 1;
  else direction_z = 0;
  if(destination_e >= current_e) direction_e = 1;
  else direction_e = 0;
  
  
  if (min_software_endstops) {
    if (destination_x < 0) destination_x = 0.0;
    if (destination_y < 0) destination_y = 0.0;
    if (destination_z < 0) destination_z = 0.0;
  }

  if (max_software_endstops) {
    if (destination_x > X_MAX_LENGTH) destination_x = X_MAX_LENGTH;
    if (destination_y > Y_MAX_LENGTH) destination_y = Y_MAX_LENGTH;
    if (destination_z > Z_MAX_LENGTH) destination_z = Z_MAX_LENGTH;
  }
  
  if(feedrate > max_feedrate) feedrate = max_feedrate;

  if(feedrate > max_z_feedrate) z_feedrate = max_z_feedrate;
  else z_feedrate = feedrate;
  
  xdiff = (destination_x - current_x);
  ydiff = (destination_y - current_y);
  zdiff = (destination_z - current_z);
  ediff = (destination_e - current_e);
  x_steps_to_take = abs(xdiff) * x_steps_per_unit;
  y_steps_to_take = abs(ydiff) * y_steps_per_unit;
  z_steps_to_take = abs(zdiff) * z_steps_per_unit;
  e_steps_to_take = abs(ediff) * e_steps_per_unit;
  if(feedrate < 10)
      feedrate = 10;
  /*
  //experimental feedrate calc
  if(abs(xdiff) > 0.1 && abs(ydiff) > 0.1)
      d = sqrt(xdiff * xdiff + ydiff * ydiff);
  else if(abs(xdiff) > 0.1)
      d = abs(xdiff);
  else if(abs(ydiff) > 0.1)
      d = abs(ydiff);
  else if(abs(zdiff) > 0.05)
      d = abs(zdiff);
  else if(abs(ediff) > 0.1)
      d = abs(ediff);
  else d = 1; //extremely slow move, should be okay for moves under 0.1mm
  time_for_move = (xdiff / (feedrate / 60000000) );
  //time = 60000000 * dist / feedrate
  //int feedz = (60000000 * zdiff) / time_for_move;
  //if(feedz > maxfeed)
  */
  #define X_TIME_FOR_MOVE ((float)x_steps_to_take / (x_steps_per_unit*feedrate/60000000))
  #define Y_TIME_FOR_MOVE ((float)y_steps_to_take / (y_steps_per_unit*feedrate/60000000))
  #define Z_TIME_FOR_MOVE ((float)z_steps_to_take / (z_steps_per_unit*z_feedrate/60000000))
  #define E_TIME_FOR_MOVE ((float)e_steps_to_take / (e_steps_per_unit*feedrate/60000000))
  
  time_for_move = max(X_TIME_FOR_MOVE, Y_TIME_FOR_MOVE);
  time_for_move = max(time_for_move, Z_TIME_FOR_MOVE);
  if(time_for_move <= 0) time_for_move = max(time_for_move, E_TIME_FOR_MOVE);

  if(x_steps_to_take) x_interval = time_for_move / x_steps_to_take * 100;
  if(y_steps_to_take) y_interval = time_for_move / y_steps_to_take * 100;
  if(z_steps_to_take) z_interval = time_for_move / z_steps_to_take * 100;
  if(e_steps_to_take && (x_steps_to_take + y_steps_to_take <= 0) ) e_interval = time_for_move / e_steps_to_take * 100;
  
  //#define DEBUGGING false
  #if 0        
  if(0) {
    Serial.print("destination_x: "); Serial.println(destination_x); 
    Serial.print("current_x: "); Serial.println(current_x); 
    Serial.print("x_steps_to_take: "); Serial.println(x_steps_to_take); 
    Serial.print("X_TIME_FOR_MVE: "); Serial.println(X_TIME_FOR_MOVE); 
    Serial.print("x_interval: "); Serial.println(x_interval); 
    Serial.println("");
    Serial.print("destination_y: "); Serial.println(destination_y); 
    Serial.print("current_y: "); Serial.println(current_y); 
    Serial.print("y_steps_to_take: "); Serial.println(y_steps_to_take); 
    Serial.print("Y_TIME_FOR_MVE: "); Serial.println(Y_TIME_FOR_MOVE); 
    Serial.print("y_interval: "); Serial.println(y_interval); 
    Serial.println("");
    Serial.print("destination_z: "); Serial.println(destination_z); 
    Serial.print("current_z: "); Serial.println(current_z); 
    Serial.print("z_steps_to_take: "); Serial.println(z_steps_to_take); 
    Serial.print("Z_TIME_FOR_MVE: "); Serial.println(Z_TIME_FOR_MOVE); 
    Serial.print("z_interval: "); Serial.println(z_interval); 
    Serial.println("");
    Serial.print("destination_e: "); Serial.println(destination_e); 
    Serial.print("current_e: "); Serial.println(current_e); 
    Serial.print("e_steps_to_take: "); Serial.println(e_steps_to_take); 
    Serial.print("E_TIME_FOR_MVE: "); Serial.println(E_TIME_FOR_MOVE); 
    Serial.print("e_interval: "); Serial.println(e_interval); 
    Serial.println("");
  }
  #endif
  
  linear_move(x_steps_to_take, y_steps_to_take, z_steps_to_take, e_steps_to_take); // make the move
}

void linear_move(unsigned long x_steps_remaining, unsigned long y_steps_remaining, unsigned long z_steps_remaining, unsigned long e_steps_remaining) // make linear move with preset speeds and destinations, see G0 and G1
{
  //Determine direction of movement
  if (destination_x > current_x) digitalWrite(X_DIR_PIN,!INVERT_X_DIR);
  else digitalWrite(X_DIR_PIN,INVERT_X_DIR);
  if (destination_y > current_y) digitalWrite(Y_DIR_PIN,!INVERT_Y_DIR);
  else digitalWrite(Y_DIR_PIN,INVERT_Y_DIR);
  if (destination_z > current_z) digitalWrite(Z_DIR_PIN,!INVERT_Z_DIR);
  else digitalWrite(Z_DIR_PIN,INVERT_Z_DIR);
  if (destination_e > current_e) digitalWrite(E_DIR_PIN,!INVERT_E_DIR);
  else digitalWrite(E_DIR_PIN,INVERT_E_DIR);
  
  if(X_MIN_PIN > -1) if(!direction_x) if(digitalRead(X_MIN_PIN) != ENDSTOPS_INVERTING) x_steps_remaining=0;
  if(Y_MIN_PIN > -1) if(!direction_y) if(digitalRead(Y_MIN_PIN) != ENDSTOPS_INVERTING) y_steps_remaining=0;
  if(Z_MIN_PIN > -1) if(!direction_z) if(digitalRead(Z_MIN_PIN) != ENDSTOPS_INVERTING) z_steps_remaining=0;
  if(X_MAX_PIN > -1) if(direction_x) if(digitalRead(X_MAX_PIN) != ENDSTOPS_INVERTING) x_steps_remaining=0;
  if(Y_MAX_PIN > -1) if(direction_y) if(digitalRead(Y_MAX_PIN) != ENDSTOPS_INVERTING) y_steps_remaining=0;
  if(Z_MAX_PIN > -1) if(direction_z) if(digitalRead(Z_MAX_PIN) != ENDSTOPS_INVERTING) z_steps_remaining=0;
  
  
  //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
  if(x_steps_remaining) enable_x();
  if(y_steps_remaining) enable_y();
  if(z_steps_remaining) { enable_z(); do_z_step(); z_steps_remaining--; }
  if(e_steps_remaining) { enable_e(); do_e_step(); e_steps_remaining--; }

    //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
  unsigned int delta_x = x_steps_remaining;
  unsigned long x_interval_nanos;
  unsigned int delta_y = y_steps_remaining;
  unsigned long y_interval_nanos;
  unsigned int delta_z = z_steps_remaining;
  unsigned long z_interval_nanos;
  boolean steep_y = delta_y > delta_x;// && delta_y > delta_e && delta_y > delta_z;
  boolean steep_x = delta_x >= delta_y;// && delta_x > delta_e && delta_x > delta_z;
  //boolean steep_z = delta_z > delta_x && delta_z > delta_y && delta_z > delta_e;
  int error_x;
  int error_y;
  int error_z;
  #ifdef RAMP_ACCELERATION
  long max_speed_steps_per_second;
  long min_speed_steps_per_second;
  #endif
  #ifdef EXP_ACCELERATION
  unsigned long virtual_full_velocity_steps;
  unsigned long full_velocity_steps;
  #endif
  unsigned long steps_remaining;
  unsigned long steps_to_take;
  
  //Do some Bresenham calculations depending on which axis will lead it.
  if(steep_y) {
   error_x = delta_y / 2;
   interval = y_interval;
   #ifdef RAMP_ACCELERATION
   max_interval = max_y_interval;
   if(e_steps_to_take > 0) steps_per_sqr_second = y_steps_per_sqr_second;
   else steps_per_sqr_second = y_travel_steps_per_sqr_second;
   max_speed_steps_per_second = 100000000 / interval;
   min_speed_steps_per_second = 100000000 / max_interval;
   float plateau_time = (max_speed_steps_per_second - min_speed_steps_per_second) / (float) steps_per_sqr_second;
   plateau_steps = (long) ((steps_per_sqr_second / 2.0 * plateau_time + min_speed_steps_per_second) * plateau_time);
   #endif
   #ifdef EXP_ACCELERATION
   if(e_steps_to_take > 0) virtual_full_velocity_steps = long_full_velocity_units * y_steps_per_unit /100;
   else virtual_full_velocity_steps = long_travel_move_full_velocity_units * y_steps_per_unit /100;
   full_velocity_steps = min(virtual_full_velocity_steps, (delta_y - y_min_constant_speed_steps) / 2);
   max_interval = max_y_interval;
   min_constant_speed_steps = y_min_constant_speed_steps;
   #endif
   steps_remaining = delta_y;
   steps_to_take = delta_y;
  } else if (steep_x) {
   error_y = delta_x / 2;
   interval = x_interval;
   #ifdef RAMP_ACCELERATION
   max_interval = max_x_interval;
   if(e_steps_to_take > 0) steps_per_sqr_second = x_steps_per_sqr_second;
   else steps_per_sqr_second = x_travel_steps_per_sqr_second;
   max_speed_steps_per_second = 100000000 / interval;
   min_speed_steps_per_second = 100000000 / max_interval;
   float plateau_time = (max_speed_steps_per_second - min_speed_steps_per_second) / (float) steps_per_sqr_second;
   plateau_steps = (long) ((steps_per_sqr_second / 2.0 * plateau_time + min_speed_steps_per_second) * plateau_time);
   #endif
   #ifdef EXP_ACCELERATION
   if(e_steps_to_take > 0) virtual_full_velocity_steps = long_full_velocity_units * x_steps_per_unit /100;
   else virtual_full_velocity_steps = long_travel_move_full_velocity_units * x_steps_per_unit /100;
   full_velocity_steps = min(virtual_full_velocity_steps, (delta_x - x_min_constant_speed_steps) / 2);
   max_interval = max_x_interval;
   min_constant_speed_steps = x_min_constant_speed_steps;
   #endif
   steps_remaining = delta_x;
   steps_to_take = delta_x;
  }
  unsigned long steps_done = 0;
  #ifdef RAMP_ACCELERATION
  plateau_steps *= 1.01; // This is to compensate we use discrete intervals
  acceleration_enabled = true;
  long full_interval = interval;
  if(interval > max_interval) acceleration_enabled = false;
  boolean decelerating = false;
  #endif
  #ifdef EXP_ACCELERATION
  acceleration_enabled = true;
  if(full_velocity_steps == 0) full_velocity_steps++;
  if(interval > max_interval) acceleration_enabled = false;
  unsigned long full_interval = interval;
  if(min_constant_speed_steps >= steps_to_take) {
    acceleration_enabled = false;
    full_interval = max(max_interval, interval); // choose the min speed between feedrate and acceleration start speed
  }
  if(full_velocity_steps < virtual_full_velocity_steps && acceleration_enabled) full_interval = max(interval,
      max_interval - ((max_interval - full_interval) * full_velocity_steps / virtual_full_velocity_steps)); // choose the min speed between feedrate and speed at full steps
  unsigned int steps_acceleration_check = 1;
  accelerating = acceleration_enabled;
  #endif
  
  unsigned long start_move_micros = micros();
  previous_micros_x = start_move_micros*100;
  previous_micros_y = previous_micros_x;
  previous_micros_z = previous_micros_x;
  previous_micros_e = previous_micros_x;
  
  //move until no more steps remain 
  while(x_steps_remaining + y_steps_remaining + z_steps_remaining + e_steps_remaining > 0) {
    //If more that HEATER_CHECK_INTERVAL ms have passed since previous heating check, adjust temp
    manage_heater();
    manage_inactivity(2);
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
    #ifdef EXP_ACCELERATION
    //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
    if (acceleration_enabled && steps_done < full_velocity_steps && steps_done / full_velocity_steps < 1 && (steps_done % steps_acceleration_check == 0)) {
      if(steps_done == 0) {
        interval = max_interval;
      } else {
        interval = max_interval - ((max_interval - full_interval) * steps_done / virtual_full_velocity_steps);
      }
    } else if (acceleration_enabled && steps_remaining < full_velocity_steps) {
      //Else, if acceleration is enabled on this move and we are in the deceleration segment, calculate the current interval
      if(steps_remaining == 0) {
        interval = max_interval;
      } else {
        interval = max_interval - ((max_interval - full_interval) * steps_remaining / virtual_full_velocity_steps);
      }
      accelerating = true;
    } else if (steps_done - full_velocity_steps >= 1 || !acceleration_enabled){
      //Else, we are just use the full speed interval as current interval
      interval = full_interval;
      accelerating = false;
    }
    #endif

    //If there are x or y steps remaining, perform Bresenham algorithm
    if(x_steps_remaining || y_steps_remaining) {
      if(X_MIN_PIN > -1) if(!direction_x) if(digitalRead(X_MIN_PIN) != ENDSTOPS_INVERTING) break;
      if(Y_MIN_PIN > -1) if(!direction_y) if(digitalRead(Y_MIN_PIN) != ENDSTOPS_INVERTING) break;
      if(X_MAX_PIN > -1) if(direction_x) if(digitalRead(X_MAX_PIN) != ENDSTOPS_INVERTING) break;
      if(Y_MAX_PIN > -1) if(direction_y) if(digitalRead(Y_MAX_PIN) != ENDSTOPS_INVERTING) break;
      if(steep_y) {
        timediff = micros() * 100 - previous_micros_y;
        while(timediff >= interval && y_steps_remaining > 0) {
          steps_done++;
          steps_remaining--;
          y_steps_remaining--; timediff -= interval;
          error_x = error_x - delta_x;
          do_y_step();
          if(error_x < 0) {
            do_x_step(); x_steps_remaining--;
            error_x = error_x + delta_y;
          }
          #ifdef RAMP_ACCELERATION
          if (steps_remaining == plateau_steps || (steps_done >= steps_to_take / 2 && accelerating && !decelerating)) break;
          #endif
          #ifdef STEP_DELAY_RATIO
          if(timediff >= interval) delayMicroseconds(long_step_delay_ratio * interval / 10000);
          #endif
          #ifdef STEP_DELAY_MICROS
          if(timediff >= interval) delayMicroseconds(STEP_DELAY_MICROS);
          #endif
        }
      } else if (steep_x) {
        timediff=micros() * 100 - previous_micros_x;
        while(timediff >= interval && x_steps_remaining>0) {
          steps_done++;
          steps_remaining--;
          x_steps_remaining--; timediff -= interval;
          error_y = error_y - delta_y;
          do_x_step();
          if(error_y < 0) { 
            do_y_step(); y_steps_remaining--;
            error_y = error_y + delta_x;
          }
          #ifdef RAMP_ACCELERATION
          if (steps_remaining == plateau_steps || (steps_done >= steps_to_take / 2 && accelerating && !decelerating)) break;
          #endif
          #ifdef STEP_DELAY_RATIO
          if(timediff >= interval) delayMicroseconds(long_step_delay_ratio * interval / 10000);
          #endif
          #ifdef STEP_DELAY_MICROS
          if(timediff >= interval) delayMicroseconds(STEP_DELAY_MICROS);
          #endif
        }
      }
    }
    #ifdef RAMP_ACCELERATION
    if((x_steps_remaining>0 || y_steps_remaining>0) &&
        steps_to_take > 0 && 
        (steps_remaining == plateau_steps || (steps_done >= steps_to_take / 2 && accelerating && !decelerating))) continue;
    #endif

    //If there are z steps remaining, check if z steps must be taken
    if(z_steps_remaining) {
      if(Z_MIN_PIN > -1) if(!direction_z) if(digitalRead(Z_MIN_PIN) != ENDSTOPS_INVERTING) break;
      if(Z_MAX_PIN > -1) if(direction_z) if(digitalRead(Z_MAX_PIN) != ENDSTOPS_INVERTING) break;
      timediff = micros() * 100-previous_micros_z;
      while(timediff >= z_interval && z_steps_remaining) {
        do_z_step();
        z_steps_remaining--;
        timediff -= z_interval;
        #ifdef STEP_DELAY_RATIO
        if(timediff >= z_interval) delayMicroseconds(long_step_delay_ratio * z_interval / 10000);
        #endif
        #ifdef STEP_DELAY_MICROS
        if(timediff >= z_interval) delayMicroseconds(STEP_DELAY_MICROS);
        #endif
      }
    }

    //If there are e steps remaining, check if e steps must be taken
    if(e_steps_remaining){
      if (x_steps_to_take + y_steps_to_take <= 0) timediff = micros()*100 - previous_micros_e;
      unsigned int final_e_steps_remaining = 0;
      if (steep_x && x_steps_to_take > 0) final_e_steps_remaining = e_steps_to_take * x_steps_remaining / x_steps_to_take;
      else if (steep_y && y_steps_to_take > 0) final_e_steps_remaining = e_steps_to_take * y_steps_remaining / y_steps_to_take;
      //If this move has X or Y steps, let E follow the Bresenham pace
      if (final_e_steps_remaining > 0)  while(e_steps_remaining > final_e_steps_remaining) { do_e_step(); e_steps_remaining--;}
      else if (x_steps_to_take + y_steps_to_take > 0)  while(e_steps_remaining) { do_e_step(); e_steps_remaining--;}
      //Else, normally check if e steps must be taken
      else while (timediff >= e_interval && e_steps_remaining) {
        do_e_step();
        e_steps_remaining--;
        timediff -= e_interval;
        #ifdef STEP_DELAY_RATIO
        if(timediff >= e_interval) delayMicroseconds(long_step_delay_ratio * e_interval / 10000);
        #endif
        #ifdef STEP_DELAY_MICROS
        if(timediff >= e_interval) delayMicroseconds(STEP_DELAY_MICROS);
        #endif
      }
    }
  }
  
  if(DISABLE_X) disable_x();
  if(DISABLE_Y) disable_y();
  if(DISABLE_Z) disable_z();
  if(DISABLE_E) disable_e();
  
  // Update current position partly based on direction, we probably can combine this with the direction code above...
  if (destination_x > current_x) current_x = current_x + x_steps_to_take / x_steps_per_unit;
  else current_x = current_x - x_steps_to_take / x_steps_per_unit;
  if (destination_y > current_y) current_y = current_y + y_steps_to_take / y_steps_per_unit;
  else current_y = current_y - y_steps_to_take / y_steps_per_unit;
  if (destination_z > current_z) current_z = current_z + z_steps_to_take / z_steps_per_unit;
  else current_z = current_z - z_steps_to_take / z_steps_per_unit;
  if (destination_e > current_e) current_e = current_e + e_steps_to_take / e_steps_per_unit;
  else current_e = current_e - e_steps_to_take / e_steps_per_unit;
}


inline void do_x_step()
{
  digitalWrite(X_STEP_PIN, HIGH);
  previous_micros_x += interval;
  //delayMicroseconds(3);
  digitalWrite(X_STEP_PIN, LOW);
}

inline void do_y_step()
{
  digitalWrite(Y_STEP_PIN, HIGH);
  previous_micros_y += interval;
  //delayMicroseconds(3);
  digitalWrite(Y_STEP_PIN, LOW);
}

inline void do_z_step()
{
  digitalWrite(Z_STEP_PIN, HIGH);
  previous_micros_z += z_interval;
  //delayMicroseconds(3);
  digitalWrite(Z_STEP_PIN, LOW);
}

inline void do_e_step()
{
  digitalWrite(E_STEP_PIN, HIGH);
  previous_micros_e += e_interval;
  //delayMicroseconds(3);
  digitalWrite(E_STEP_PIN, LOW);
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
