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

#ifdef PIDTEMP
 extern unsigned int PID_Kp, PID_Ki, PID_Kd;
#endif


#ifdef USE_EEPROM_SETTINGS

//======================================================================================
//========================= Read / Write EEPROM =======================================
template <class T> int EEPROM_write_setting(int address, const T& value)
{
  const byte* p = (const byte*)(const void*)&value;
  int i;
  for (i = 0; i < (int)sizeof(value); i++)
    eeprom_write_byte((unsigned char *)address++, *p++);
  return i;
}

template <class T> int EEPROM_read_setting(int address, T& value)
{
  byte* p = (byte*)(void*)&value;
  int i;
  for (i = 0; i < (int)sizeof(value); i++)
    *p++ = eeprom_read_byte((unsigned char *)address++);
  return i;
}
//======================================================================================


void EEPROM_StoreSettings() 
{
  char ver[4]= "000";
  EEPROM_write_setting(EEPROM_OFFSET, ver); // invalidate data first
  EEPROM_write_setting(axis_steps_per_unit_address, axis_steps_per_unit);
  EEPROM_write_setting(max_feedrate_address, max_feedrate);
  EEPROM_write_setting(max_acceleration_units_per_sq_second_address, max_acceleration_units_per_sq_second);
  EEPROM_write_setting(move_acceleration_address, move_acceleration);
  EEPROM_write_setting(retract_acceleration_address, retract_acceleration);
  EEPROM_write_setting(minimumfeedrate_address, minimumfeedrate);
  EEPROM_write_setting(mintravelfeedrate_address, mintravelfeedrate);
  EEPROM_write_setting(min_seg_time_address, min_seg_time);  //Min Segment Time, not used yet
  EEPROM_write_setting(max_xy_jerk_address, max_xy_jerk);
  EEPROM_write_setting(max_z_jerk_address, max_z_jerk);
  EEPROM_write_setting(max_e_jerk_address, max_e_jerk);

  //PID Settings
  #ifdef PIDTEMP
   EEPROM_write_setting(Kp_address, PID_Kp);     //Kp
   EEPROM_write_setting(Ki_address, PID_Ki);     //Ki
   EEPROM_write_setting(Kd_address, PID_Kd);     //Kd
  #else
   EEPROM_write_setting(Kp_address, 2048);     //Kp
   EEPROM_write_setting(Ki_address, 32);     //Ki
   EEPROM_write_setting(Kd_address, 2048);     //Kd
  #endif
  

  char ver2[4]=EEPROM_VERSION;
  EEPROM_write_setting(EEPROM_OFFSET, ver2); // validate data
  showString(PSTR("Settings Stored\r\n"));
 
}


void EEPROM_printSettings()
{  
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
      showString(PSTR("  M202 X"));
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

      showString(PSTR("Advanced variables (mm/s): S=Min feedrate, T=Min travel feedrate, X=max xY jerk,  Z=max Z jerk, E=max E jerk\r\n"));

      showString(PSTR("  M205 S"));
      Serial.print(minimumfeedrate ); 
      showString(PSTR(" T" ));
      Serial.print(mintravelfeedrate ); 
//      showString(PSTR(" B"));
//      Serial.print(min_seg_time ); 
      showString(PSTR(" X"));
      Serial.print(max_xy_jerk ); 
      showString(PSTR(" Z"));
      Serial.print(max_z_jerk);
      showString(PSTR(" E"));
      Serial.println(max_e_jerk);

      
    #ifdef PIDTEMP
    
      showString(PSTR("PID settings:\r\n"));
      showString(PSTR("  M301 P"));
      Serial.print(PID_Kp); 
      showString(PSTR(" I"));
      Serial.print(PID_Ki); 
      showString(PSTR(" D"));
      Serial.println(PID_Kd);
    
    #endif
  #endif

} 


void EEPROM_RetrieveSettings(bool def, bool printout)
{  // if def=true, the default values will be used

    int i=EEPROM_OFFSET;
    char stored_ver[4];
    char ver[4]=EEPROM_VERSION;
    
    EEPROM_read_setting(EEPROM_OFFSET,stored_ver); //read stored version
    if ((!def)&&(strncmp(ver,stored_ver,3)==0))
    {   // version number match
      EEPROM_read_setting(axis_steps_per_unit_address, axis_steps_per_unit);
      EEPROM_read_setting(max_feedrate_address, max_feedrate);
      EEPROM_read_setting(max_acceleration_units_per_sq_second_address, max_acceleration_units_per_sq_second);
      EEPROM_read_setting(move_acceleration_address, move_acceleration);
      EEPROM_read_setting(retract_acceleration_address, retract_acceleration);
      EEPROM_read_setting(minimumfeedrate_address, minimumfeedrate);
      EEPROM_read_setting(mintravelfeedrate_address, mintravelfeedrate);
      EEPROM_read_setting(min_seg_time_address, min_seg_time);  //min Segmenttime --> not used yet
      EEPROM_read_setting(max_xy_jerk_address, max_xy_jerk);
      EEPROM_read_setting(max_z_jerk_address, max_z_jerk);
      EEPROM_read_setting(max_e_jerk_address, max_e_jerk);

      #ifdef PIDTEMP
       EEPROM_read_setting(Kp_address, PID_Kp);
       EEPROM_read_setting(Ki_address, PID_Ki);
       EEPROM_read_setting(Kd_address, PID_Kd);
      #endif

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
      max_e_jerk=_MAX_E_JERK;
      min_seg_time=_MIN_SEG_TIME;
      
      #ifdef PIDTEMP
       PID_Kp = PID_PGAIN;
       PID_Ki = PID_IGAIN;
       PID_Kd = PID_DGAIN;
      #endif

      showString(PSTR("Using Default settings\r\n"));
    }
    
    if(printout)
    {
      EEPROM_printSettings();
    }
}  

#endif
