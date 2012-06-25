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


#ifndef __EEPROMH
#define __EEPROMH

#define EEPROM_OFFSET 100


// IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
// in the functions below, also increment the version number. This makes sure that
// the default values are used whenever there is a change to the data, to prevent
// wrong data being written to the variables.
// ALSO:  always make sure the variables in the Store and retrieve sections are in the same order.
#define EEPROM_VERSION "S03"


extern float axis_steps_per_unit[4]; 
extern float max_feedrate[4];
extern long  max_acceleration_units_per_sq_second[4];
extern float move_acceleration;
extern float retract_acceleration;
extern float mintravelfeedrate;
extern float minimumfeedrate;
extern float max_xy_jerk;
extern float max_z_jerk;
extern float max_e_jerk;
extern unsigned long min_seg_time;
extern unsigned int Kp, Ki, Kd;

#define axis_steps_per_unit_address                     (EEPROM_OFFSET +  4)
#define max_feedrate_address                            (EEPROM_OFFSET + 20)
#define max_acceleration_units_per_sq_second_address    (EEPROM_OFFSET + 36)
#define move_acceleration_address                       (EEPROM_OFFSET + 52)
#define retract_acceleration_address                    (EEPROM_OFFSET + 56)
#define mintravelfeedrate_address                       (EEPROM_OFFSET + 60)
#define minimumfeedrate_address                         (EEPROM_OFFSET + 64)
#define max_xy_jerk_address                             (EEPROM_OFFSET + 68)
#define max_z_jerk_address                              (EEPROM_OFFSET + 72)
#define max_e_jerk_address                              (EEPROM_OFFSET + 76)
#define min_seg_time_address                            (EEPROM_OFFSET + 80)
#define Kp_address                                      (EEPROM_OFFSET + 84)
#define Ki_address                                      (EEPROM_OFFSET + 86)
#define Kd_address                                      (EEPROM_OFFSET + 88)

extern void EEPROM_RetrieveSettings(bool def, bool printout );
extern void EEPROM_printSettings();
extern void EEPROM_StoreSettings();


#endif
