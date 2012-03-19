/*
 EEPROM routines to save Sprinter Settings 
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>. 
*/

#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include "Sprinter.h"
#include "store_eeprom.h"
#include "Configuration.h"



#ifdef USE_EEPROM_SETTINGS

//======================================================================================
//========================= Read / Write EEPROM =======================================
template <class T> int EEPROM_writeAnything(int &ee, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < (int)sizeof(value); i++)
    eeprom_write_byte((unsigned char *)ee++, *p++);
  return i;
}

template <class T> int EEPROM_readAnything(int &ee, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < (int)sizeof(value); i++)
    *p++ = eeprom_read_byte((unsigned char *)ee++);
  return i;
}
//======================================================================================


void EEPROM_StoreSettings() 
{

  unsigned long ul_help = 20000;
  unsigned int ui_help = 0;
  char ver[4]= "000";
  int i=EEPROM_OFFSET;
  EEPROM_writeAnything(i,ver); // invalidate data first 
  EEPROM_writeAnything(i,axis_steps_per_unit);  
  EEPROM_writeAnything(i,max_feedrate);  
  EEPROM_writeAnything(i,max_acceleration_units_per_sq_second);
  EEPROM_writeAnything(i,move_acceleration);
  EEPROM_writeAnything(i,retract_acceleration);
  EEPROM_writeAnything(i,minimumfeedrate);
  EEPROM_writeAnything(i,mintravelfeedrate);
  EEPROM_writeAnything(i,ul_help);  //Min Segment Time, not used yet
  EEPROM_writeAnything(i,max_xy_jerk);
  EEPROM_writeAnything(i,max_z_jerk);

  //PID Settings, not used yet --> placeholder
  ui_help = 2560;
  EEPROM_writeAnything(i,ui_help);     //Kp
  ui_help = 64;
  EEPROM_writeAnything(i,ui_help);     //Ki
  ui_help = 4096;
  EEPROM_writeAnything(i,ui_help);     //Kd

  char ver2[4]=EEPROM_VERSION;
  i=EEPROM_OFFSET;
  EEPROM_writeAnything(i,ver2); // validate data
  showString(PSTR("Settings Stored\r\n"));
 
}


void EEPROM_printSettings()
{  
      // if def=true, the default values will be used
  #ifdef PRINT_EEPROM_SETTING
      showString(PSTR("Steps per unit:\r\n"));
      showString(PSTR(" M92 X"));
      Serial.print(axis_steps_per_unit[0]);
      showString(PSTR(" Y"));
      Serial.print(axis_steps_per_unit[1]);
      showString(PSTR(" Z"));
      Serial.print(axis_steps_per_unit[2]);
      showString(PSTR(" E"));
      Serial.println(axis_steps_per_unit[3]);
      
      showString(PSTR("Maximum feedrates (mm/s):\r\n"));
      showString(PSTR("  M203 X"));
      Serial.print(max_feedrate[0]);
      showString(PSTR(" Y"));
      Serial.print(max_feedrate[1]); 
      showString(PSTR(" Z"));
      Serial.print(max_feedrate[2]); 
      showString(PSTR(" E"));
      Serial.println(max_feedrate[3]);

      showString(PSTR("Maximum Acceleration (mm/s2):\r\n"));
      showString(PSTR("  M201 X"));
      Serial.print(max_acceleration_units_per_sq_second[0] ); 
      showString(PSTR(" Y"));
      Serial.print(max_acceleration_units_per_sq_second[1] ); 
      showString(PSTR(" Z"));
      Serial.print(max_acceleration_units_per_sq_second[2] );
      showString(PSTR(" E"));
      Serial.println(max_acceleration_units_per_sq_second[3]);

      showString(PSTR("Acceleration: S=acceleration, T=retract acceleration\r\n"));
      showString(PSTR("  M204 S"));
      Serial.print(move_acceleration ); 
      showString(PSTR(" T"));
      Serial.println(retract_acceleration);

      showString(PSTR("Advanced variables: S=Min feedrate (mm/s), T=Min travel feedrate (mm/s), X=maximum xY jerk (mm/s),  Z=maximum Z jerk (mm/s)\r\n"));

      showString(PSTR("  M205 S"));
      Serial.print(minimumfeedrate ); 
      showString(PSTR(" T" ));
      Serial.print(mintravelfeedrate ); 
//      showString(PSTR(" B"));
//      Serial.print(minsegmenttime ); 
      showString(PSTR(" X"));
      Serial.print(max_xy_jerk ); 
      showString(PSTR(" Z"));
      Serial.println(max_z_jerk);
      
    #ifdef PIDTEMP
    /*
      showString(PSTR("PID settings:");
      showString(PSTR("   M301 P"));
      Serial.print(Kp); 
      showString(PSTR(" I"));
      Serial.print(Ki); 
      SshowString(PSTR(" D"));
      Serial.print(Kd);
    */
    #endif
  #endif

} 


void EEPROM_RetrieveSettings(bool def, bool printout)
{  // if def=true, the default values will be used

    int i=EEPROM_OFFSET;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;
    unsigned long ul_help = 0;
    
    EEPROM_readAnything(i,stored_ver); //read stored version
    if ((!def)&&(strncmp(ver,stored_ver,3)==0)) 
    {   // version number match
      EEPROM_readAnything(i,axis_steps_per_unit);  
      EEPROM_readAnything(i,max_feedrate);  
      EEPROM_readAnything(i,max_acceleration_units_per_sq_second);
      EEPROM_readAnything(i,move_acceleration);
      EEPROM_readAnything(i,retract_acceleration);
      EEPROM_readAnything(i,minimumfeedrate);
      EEPROM_readAnything(i,mintravelfeedrate);
      EEPROM_readAnything(i,ul_help);  //min Segmenttime --> not used yet
      EEPROM_readAnything(i,max_xy_jerk);
      EEPROM_readAnything(i,max_z_jerk);

      unsigned int Kp,Ki,Kd;
      EEPROM_readAnything(i,Kp);
      EEPROM_readAnything(i,Ki);
      EEPROM_readAnything(i,Kd);

      showString(PSTR("Stored settings retreived\r\n"));
    }
    else 
    {

      float tmp1[]=_AXIS_STEP_PER_UNIT;
      float tmp2[]=_MAX_FEEDRATE;
      long tmp3[]=_MAX_ACCELERATION_UNITS_PER_SQ_SECOND;
      for (short i=0;i<4;i++) 
      {
        axis_steps_per_unit[i]=tmp1[i];  
        max_feedrate[i]=tmp2[i];  
        max_acceleration_units_per_sq_second[i]=tmp3[i];
      }
      move_acceleration=_ACCELERATION;
      retract_acceleration=_RETRACT_ACCELERATION;
      minimumfeedrate=DEFAULT_MINIMUMFEEDRATE;
      mintravelfeedrate=DEFAULT_MINTRAVELFEEDRATE;
      max_xy_jerk=_MAX_XY_JERK;
      max_z_jerk=_MAX_Z_JERK;

      showString(PSTR("Using Default settings\r\n"));
    }

    if(printout)
    {
      EEPROM_printSettings();
    }
}  

#endif
