// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL

#include "Tonokip_Firmware.h"
#include "configuration.h"
#include "pins.h"
#include "Axis.h"

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
// M86  - If Endstop is Not Activated then Abort Print. Specify X and/or Y
// M92  - Set axis_steps_per_unit - same syntax as G92
// M114 - Report current location
// M115	- Capabilities string
// M140 - Set bed target temp
// M190 - Wait for bed current temp to reach target temp.
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000)


// Newstyle Axis
Axis AXIS[NUM_AXIS] = 
{ 
	Axis(X_STEP_PIN,X_DIR_PIN,X_ENABLE_PIN,X_MIN_PIN,X_MAX_PIN,X_STEPS_PER_UNIT,X_ENABLE_ON,INVERT_X_DIR,X_MAX_LENGTH,X_MAX_FEED,X_HOME_DIR),
	Axis(Y_STEP_PIN,Y_DIR_PIN,Y_ENABLE_PIN,Y_MIN_PIN,Y_MAX_PIN,Y_STEPS_PER_UNIT,Y_ENABLE_ON,INVERT_Y_DIR,Y_MAX_LENGTH,Y_MAX_FEED,Y_HOME_DIR),
	Axis(Z_STEP_PIN,Z_DIR_PIN,Z_ENABLE_PIN,Z_MIN_PIN,Z_MAX_PIN,Z_STEPS_PER_UNIT,Z_ENABLE_ON,INVERT_Z_DIR,Z_MAX_LENGTH,Z_MAX_FEED,Z_HOME_DIR),
	Axis(E_STEP_PIN,E_DIR_PIN,E_ENABLE_PIN,-1,-1,E_STEPS_PER_UNIT,E_ENABLE_ON,INVERT_E_DIR,E_MAX_LENGTH,E_MAX_FEED,0)
};

//Stepper Movement Variables
unsigned long previous_millis_heater, previous_millis_bed_heater;
boolean acceleration_enabled = false, accelerating = false;
unsigned long interval;
float feedrate = 1500;
long gcode_N, gcode_LastN;
long timediff = 0;


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
  
  manage_heater();
  
  manage_inactivity(1); //shutdown if not receiving any new commands
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

#ifdef SDSUPPORT
  char *starpos = NULL;
#endif

  if(code_seen('G'))
  {
    switch((int)code_value())
    {
      case 0: // G0 -> G1
      case 1: // G1
        get_coordinates(); // For X Y Z E F
        prepare_move();
        previous_millis_cmd = millis();
        return;
      case 4: // G4 dwell
        codenum = 0;
        if(code_seen('P')) codenum = code_value(); // milliseconds to wait
        if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
        previous_millis_heater = millis();  // keep track of when we started waiting
        while((millis() - previous_millis_heater) < codenum ) manage_heater(); //manage heater until time is up
        break;
      case 28: //G28 Home all Axis one at a time
        for(int ax=0;ax<NUM_AXIS;ax++)
          AXIS[ax].home();
        previous_millis_cmd = millis();
        break;
      case 90: // G90
	for(int ax=0;ax<NUM_AXIS;ax++)
		AXIS[ax].relative = false;
        break;
      case 91: // G91
	for(int ax=0;ax<NUM_AXIS;ax++)
		AXIS[ax].relative = true;
        break;
      case 92: // G92
        if(code_seen('X')) AXIS[0].current = code_value();
        if(code_seen('Y')) AXIS[1].current = code_value();
        if(code_seen('Z')) AXIS[2].current = code_value();
        if(code_seen('E')) AXIS[3].current = code_value();
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
        previous_millis_heater = millis(); 
        while(current_raw < target_raw) {
          if( (millis() - previous_millis_heater) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            Serial.print("T:");
            Serial.println( analog2temp(current_raw) ); 
            previous_millis_heater = millis(); 
          }
          manage_heater();
        }
        break;
      case 190: // M190 - Wait bed for heater to reach target.
      #if TEMP_1_PIN > -1
        if (code_seen('S')) target_bed_raw = temp2analog(code_value());
        previous_millis_heater = millis(); 
        while(current_bed_raw < target_bed_raw) {
          if( (millis()-previous_millis_heater) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            tt=analog2temp(current_raw);
            Serial.print("T:");
            Serial.println( tt );
            Serial.print("ok T:");
            Serial.print( tt ); 
            Serial.print(" B:");
            Serial.println( analog2temp(current_bed_raw) ); 
            previous_millis_heater = millis(); 
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
        AXIS[3].relative = false;
        break;
      case 83:
        AXIS[3].relative = true;
        break;
      case 84:
        if(code_seen('S')){ stepper_inactive_time = code_value() * 1000; }
        else{ ; }
        break;
      case 85: // M85
        code_seen('S');
        max_inactive_time = code_value() * 1000; 
        break;
      case 86: // M86 If Endstop is Not Activated then Abort Print
	;
        break;
      case 92: // M92
        if(code_seen('X')) AXIS[0].steps_per_unit = code_value();
        if(code_seen('Y')) AXIS[1].steps_per_unit = code_value();
        if(code_seen('Z')) AXIS[2].steps_per_unit = code_value();
        if(code_seen('E')) AXIS[3].steps_per_unit = code_value();
        break;
      case 115: // M115
        Serial.println("FIRMWARE_NAME:Sprinter FIRMWARE_URL:http%%3A/github.com/kliment/Sprinter/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1");
        break;
      case 114: // M114
	Serial.print("X:");
        Serial.print(AXIS[0].current);
	Serial.print("Y:");
        Serial.print(AXIS[1].current);
	Serial.print("Z:");
        Serial.print(AXIS[2].current);
	Serial.print("E:");
        Serial.println(AXIS[3].current);
        break;
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
  if(code_seen('X')) AXIS[0].set_target((float)code_value());
  if(code_seen('Y')) AXIS[1].set_target((float)code_value());
  if(code_seen('Z')) AXIS[2].set_target((float)code_value());
  if(code_seen('E')) AXIS[3].set_target((float)code_value());
  if(code_seen('F')) {
    float next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

void prepare_move()
{
  // Determine which axis will take the longest time.
  Serial.print("Prepare: "); Serial.println(feedrate);
  unsigned long time_for_move = 0;
  for(int ax=0;ax < NUM_AXIS;ax++) 
  {
    unsigned long axtime = AXIS[ax].get_time_for_move(feedrate);
    if(axtime > time_for_move) time_for_move =  axtime;
  }

  // Inform all axis how long the slowpoke takes.
  for(int ax=0;ax<NUM_AXIS;ax++) AXIS[ax].set_time_for_move(time_for_move);

  // Keep on truckin'
  linear_move(); // make the move
}

void linear_move() // make linear move with preset speeds and destinations, see G0 and G1
{
  previous_millis_heater = millis();
  
  unsigned long deltas[NUM_AXIS];
  long errors[NUM_AXIS];
  unsigned long interval        = 0;
  int primary_axis = 0;
  unsigned long primary_axis_steps = 0;
  for(int ax=0;ax<NUM_AXIS;ax++)
  {
    AXIS[ax].precomputemove(); 
    deltas[ax] = AXIS[ax].steps_remaining;
    if(AXIS[ax].steps_remaining > primary_axis_steps)
    {
      primary_axis = ax;
      primary_axis_steps = AXIS[ax].steps_remaining;
    }
  }

  for(int ax=0;ax<NUM_AXIS;ax++)
    errors[ax] = AXIS[primary_axis].steps_remaining / 2;
    
  interval = AXIS[primary_axis].interval;

  Serial.print("PA: ");Serial.print(primary_axis);
  Serial.print(" int: ");Serial.println(interval);

  AXIS[primary_axis].precompute_accel(interval, deltas[primary_axis]);

	unsigned long previous_nanos = micros() * 100l;
  unsigned long timediff = 0;
  while(axis_are_moving())
  {
    // TODO: tht timer can and will wrap around.
    unsigned long now_nanos = micros() * 100l;
    timediff += now_nanos - previous_nanos;
    previous_nanos = now_nanos;

    interval = AXIS[primary_axis].recompute_accel(timediff, interval);
    while(timediff >= interval && axis_are_moving())
    {
      timediff -= interval;
      for(int ax=0;ax<NUM_AXIS;ax++)
      {
        if(ax == primary_axis)
        {
          AXIS[ax].do_step();
          continue;
        }
        errors[ax] = errors[ax] - deltas[ax];
        if(errors[ax] < 0)
        {
          AXIS[ax].do_step();
          errors[ax] = errors[ax] + deltas[primary_axis];
        }
      }
    }
        
    //If more that 50ms have passed since previous heating check, adjust temp
    if((millis() - previous_millis_heater) >= 50 ) 
    {
      manage_heater();
      previous_millis_heater = millis();
      manage_inactivity(2);
    }
  }
}

bool axis_are_moving()
{
	for(int ax=0;ax<NUM_AXIS;ax++)
		if(AXIS[ax].is_moving()) return true;
	return false;
}


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
    if(current_raw > maxttemp) {
        // We are too hot. Emergency brake to protect hotend
        kill(5);
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
  
  if(millis() - previous_millis_bed_heater < 5000)
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

inline void kill(byte debug)
{
  if(HEATER_0_PIN > -1) digitalWrite(HEATER_0_PIN,LOW);
  if(HEATER_1_PIN > -1) digitalWrite(HEATER_1_PIN,LOW);
  
  if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT);
  
  while(1)
  {
    switch(debug)
    {
      case 1: Serial.print("Inactivity Shutdown, Last Line: "); break;
      case 2: Serial.print("Linear Move Abort, Last Line: "); break;
      case 3: Serial.print("Homing X Min Stop Fail, Last Line: "); break;
      case 4: Serial.print("Homing Y Min Stop Fail, Last Line: "); break;
      case 5: Serial.print("Hot-end overheat protection, Last Line: "); break;
    } 
    Serial.println(gcode_LastN);
    delay(5000); // 5 Second delay
  }
}

inline void manage_inactivity(byte debug) { 
if( (millis()-previous_millis_cmd) >  max_inactive_time ) if(max_inactive_time) kill(debug); 
}
