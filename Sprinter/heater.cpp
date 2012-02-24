/*
 Reprap heater funtions based on Sprinter

 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>. */

/*
 This softwarepart for Heatercontrol is based on Sprinter
 big thanks to kliment (https://github.com/kliment/Sprinter)
*/


#include <avr/pgmspace.h>

#include "heater.h"
#include "fastio.h"
#include "pins.h"
#include "Sprinter.h"

#ifdef CONTROLLERFAN_PIN
  void controllerFan(void);
#endif

// Manage heater variables. For a thermistor or AD595 thermocouple, raw values refer to the 
// reading from the analog pin. For a MAX6675 thermocouple, the raw value is the temperature in 0.25 
// degree increments (i.e. 100=25 deg). 

int target_raw = 0;
int target_temp = 0;
int current_raw = 0;
int current_raw_maxval = -32000;
int current_raw_minval = 32000;
int tt_maxval;
int tt_minval;
int target_bed_raw = 0;
int current_bed_raw = 0;
unsigned long previous_millis_heater, previous_millis_bed_heater, previous_millis_monitor;

#ifdef PIDTEMP
  volatile unsigned char g_heater_pwm_val = 0;
 
  unsigned char PWM_off_time = 0;
  unsigned char PWM_out_on = 0;
  
  int temp_iState = 0;
  int temp_dState = 0;
  int prev_temp = 0;
  int pTerm;
  int iTerm;
  int dTerm;
      //int output;
  int error;
  int heater_duty = 0;
  const int temp_iState_min = 256L * -PID_INTEGRAL_DRIVE_MAX / PID_IGAIN;
  const int temp_iState_max = 256L * PID_INTEGRAL_DRIVE_MAX / PID_IGAIN;
#endif


#ifdef AUTOTEMP
    float autotemp_max=AUTO_TEMP_MAX;
    float autotemp_min=AUTO_TEMP_MIN;
    float autotemp_factor=AUTO_TEMP_FACTOR;
    int   autotemp_setpoint=0;
    bool autotemp_enabled=true;
#endif

#ifndef HEATER_CURRENT
  #define HEATER_CURRENT 255
#endif

#ifdef SMOOTHING
  uint32_t nma = 0;
#endif

#ifdef WATCHPERIOD
  int watch_raw = -1000;
  unsigned long watchmillis = 0;
#endif

#ifdef MINTEMP
  int minttemp = temp2analogh(MINTEMP);
#endif

#ifdef MAXTEMP
  int maxttemp = temp2analogh(MAXTEMP);
#endif



#define HEAT_INTERVAL 250
#ifdef HEATER_USES_MAX6675
unsigned long max6675_previous_millis = 0;
int max6675_temp = 2000;

int read_max6675()
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
  WRITE(MAX6675_SS, 0);
  
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
  WRITE(MAX6675_SS, 1);

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


#ifdef PID_SOFT_PWM

 void init_Timer2_softpwm(void)
 {
  // This is a simple SOFT PWM with 500 Hz for Extruder Heating

 
  TIFR2 = (1 << TOV2);          // clear interrupt flag
  TCCR2B = (1 << CS22) | (1 << CS20);         // start timer (ck/128 prescalar)
  TCCR2A = (1 << WGM21);        // CTC mode
  OCR2A = 128;                  // We want to have at least 30Hz or else it gets choppy
  TIMSK2 = (1 << OCIE2A);       // enable timer2 output compare match interrupt

 }


 ISR(TIMER2_COMPA_vect)
 {

    
    if(g_heater_pwm_val < 2)
    {
      #if LED_PIN > -1
        WRITE(LED_PIN,LOW);
      #endif
      WRITE(HEATER_0_PIN,LOW);
      PWM_out_on = 0;
      OCR2A = 128; 
    }
    else if(g_heater_pwm_val > 253)
    {
      #if LED_PIN > -1
        WRITE(LED_PIN,HIGH);
      #endif
      WRITE(HEATER_0_PIN,HIGH);
      PWM_out_on = 1;
      OCR2A = 128; 
    }
    else
    {

       if(PWM_out_on == 1)
       {

         #if LED_PIN > -1
           WRITE(LED_PIN,LOW);
         #endif
         WRITE(HEATER_0_PIN,LOW);
         PWM_out_on = 0;
         OCR2A = PWM_off_time;
       }
       else
       {

         #if LED_PIN > -1
           WRITE(LED_PIN,HIGH);
         #endif
         WRITE(HEATER_0_PIN,HIGH);
         PWM_out_on = 1;
         
         if(g_heater_pwm_val > 253)
         {
           OCR2A = 253;
           PWM_off_time = 2;
         }
         else if(g_heater_pwm_val < 2)
         {
           OCR2A = 2;
           PWM_off_time = 253;
         }
         else
         {
           OCR2A =  g_heater_pwm_val;
           PWM_off_time = 255 - g_heater_pwm_val;
         }
         
       }
    }
    
  
 }
 #endif
 
 
 
 void manage_heater()
 {

  //Temperatur Monitor for repetier
  if((millis() - previous_millis_monitor) > 250 )
  {
    previous_millis_monitor = millis();
    if(manage_monitor <= 1)
    {
      showString(PSTR("MTEMP:"));
      Serial.print(millis());
      if(manage_monitor<1)
      {
        showString(PSTR(" "));
        Serial.print(analog2temp(current_raw));
        showString(PSTR(" "));
        Serial.print(target_temp);
        showString(PSTR(" "));
        #ifdef PIDTEMP
        Serial.println(heater_duty);
        #else 
          #if (HEATER_0_PIN > -1)
          if(READ(HEATER_0_PIN))
            Serial.println(255);
          else
            Serial.println(0);
          #else
          Serial.println(0);
          #endif
        #endif
      }
      #if THERMISTORBED!=0
      else
      {
        showString(PSTR(" "));
        Serial.print(analog2tempBed(current_bed_raw));
        showString(PSTR(" "));
        Serial.print(analog2tempBed(target_bed_raw));
        showString(PSTR(" "));
        #if (HEATER_1_PIN > -1)
          if(READ(HEATER_1_PIN))
            Serial.println(255);
          else
            Serial.println(0);
        #else
          Serial.println(0);
        #endif  
      }
      #endif
      
    }
  
  }
  // ENDE Temperatur Monitor for repetier
 
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
  
  //MIN / MAX save to display the jitter of Heaterbarrel
  if(current_raw > current_raw_maxval)
    current_raw_maxval = current_raw;
    
  if(current_raw < current_raw_minval)
    current_raw_minval = current_raw;
 
  #ifdef SMOOTHING
    if (!nma) nma = SMOOTHFACTOR * current_raw;
    nma = (nma + current_raw) - (nma / SMOOTHFACTOR);
    current_raw = nma / SMOOTHFACTOR;
  #endif
  
  #ifdef WATCHPERIOD
    if(watchmillis && millis() - watchmillis > WATCHPERIOD)
    {
        if(watch_raw + 1 >= current_raw)
        {
            target_temp = target_raw = 0;
            WRITE(HEATER_0_PIN,LOW);

            #ifdef PID_SOFT_PWM
              g_heater_pwm_val = 0;           
            #else
              analogWrite(HEATER_0_PIN, 0);
              #if LED_PIN>-1
                WRITE(LED_PIN,LOW);
              #endif
            #endif
        }
        else
        {
            watchmillis = 0;
        }
    }
  #endif
  
  //If tmp is lower then MINTEMP stop the Heater
  //or it os better to deaktivate the uutput PIN or PWM ?
  #ifdef MINTEMP
    if(current_raw <= minttemp)
        target_temp = target_raw = 0;
  #endif
  
  #ifdef MAXTEMP
    if(current_raw >= maxttemp)
    {
        target_temp = target_raw = 0;
    
        #if (ALARM_PIN > -1) 
          WRITE(ALARM_PIN,HIGH);
        #endif
    }
  #endif

  #if (TEMP_0_PIN > -1) || defined (HEATER_USES_MAX6675) || defined (HEATER_USES_AD595)
    #ifdef PIDTEMP
      
      int current_temp = analog2temp(current_raw);
      error = target_temp - current_temp;
      int delta_temp = current_temp - prev_temp;
      
      prev_temp = current_temp;
      pTerm = ((long)PID_PGAIN * error) / 256;
      const int H0 = min(HEATER_DUTY_FOR_SETPOINT(target_temp),HEATER_CURRENT);
      heater_duty = H0 + pTerm;
      
      if(error < 30)
      {
        temp_iState += error;
        temp_iState = constrain(temp_iState, temp_iState_min, temp_iState_max);
        iTerm = ((long)PID_IGAIN * temp_iState) / 256;
        heater_duty += iTerm;
      }
      
      int prev_error = abs(target_temp - prev_temp);
      int log3 = 1; // discrete logarithm base 3, plus 1
      
      if(prev_error > 81){ prev_error /= 81; log3 += 4; }
      if(prev_error >  9){ prev_error /=  9; log3 += 2; }
      if(prev_error >  3){ prev_error /=  3; log3 ++;   }
      
      dTerm = ((long)PID_DGAIN * delta_temp) / (256*log3);
      heater_duty += dTerm;
      heater_duty = constrain(heater_duty, 0, HEATER_CURRENT);

      #ifdef PID_SOFT_PWM
        g_heater_pwm_val = (unsigned char)heater_duty;
      #else
        analogWrite(HEATER_0_PIN, heater_duty);
    
        #if LED_PIN>-1
          analogWrite(LED_PIN, constrain(LED_PWM_FOR_BRIGHTNESS(heater_duty),0,255));
        #endif
      #endif
  
    #else
    
      if(current_raw >= target_raw)
      {
        WRITE(HEATER_0_PIN,LOW);
        #if LED_PIN>-1
            WRITE(LED_PIN,LOW);
        #endif
      }
      else 
      {
        WRITE(HEATER_0_PIN,HIGH);
        #if LED_PIN > -1
            WRITE(LED_PIN,HIGH);
        #endif
      }
    #endif
  #endif
    
  if(millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
    return;
  
  previous_millis_bed_heater = millis();

  #ifndef TEMP_1_PIN
    return;
  #endif

  #if TEMP_1_PIN == -1
    return;
  #else
  
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
  
  
  #ifdef MINTEMP
    if(current_bed_raw >= target_bed_raw || current_bed_raw < minttemp)
  #else
    if(current_bed_raw >= target_bed_raw)
  #endif
    {
      WRITE(HEATER_1_PIN,LOW);
    }
    else 
    {
      WRITE(HEATER_1_PIN,HIGH);
    }
    #endif
    
#ifdef CONTROLLERFAN_PIN
  controllerFan(); //Check if fan should be turned on to cool stepper drivers down
#endif

}

#if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
int temp2analog_thermistor(int celsius, const short table[][2], int numtemps) 
{
    int raw = 0;
    byte i;
    
    for (i=1; i<numtemps; i++)
    {
      if (table[i][1] < celsius)
      {
        raw = table[i-1][0] + 
          (celsius - table[i-1][1]) * 
          (table[i][0] - table[i-1][0]) /
          (table[i][1] - table[i-1][1]);
      
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == numtemps) raw = table[i-1][0];

    return 1023 - raw;
}
#endif

#if defined (HEATER_USES_AD595) || defined (BED_USES_AD595)
int temp2analog_ad595(int celsius) 
{
    return celsius * 1024 / (500);
}
#endif

#if defined (HEATER_USES_MAX6675) || defined (BED_USES_MAX6675)
int temp2analog_max6675(int celsius) 
{
    return celsius * 4;
}
#endif

#if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
int analog2temp_thermistor(int raw,const short table[][2], int numtemps) {
    int celsius = 0;
    byte i;
    
    raw = 1023 - raw;

    for (i=1; i<numtemps; i++)
    {
      if (table[i][0] > raw)
      {
        celsius  = table[i-1][1] + 
          (raw - table[i-1][0]) * 
          (table[i][1] - table[i-1][1]) /
          (table[i][0] - table[i-1][0]);

        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == numtemps) celsius = table[i-1][1];

    return celsius;
}
#endif

#if defined (HEATER_USES_AD595) || defined (BED_USES_AD595)
int analog2temp_ad595(int raw)
{
        return raw * 500 / 1024;
}
#endif

#if defined (HEATER_USES_MAX6675) || defined (BED_USES_MAX6675)
int analog2temp_max6675(int raw)
{
    return raw / 4;
}
#endif

#ifdef CONTROLLERFAN_PIN
unsigned long lastMotor = 0; //Save the time for when a motor was turned on last
unsigned long lastMotorCheck = 0;

void controllerFan()
{  
  if ((millis() - lastMotorCheck) >= 2500) //Not a time critical function, so we only check every 2500ms
  {
    lastMotorCheck = millis();
    
    if(!READ(X_ENABLE_PIN) || !READ(Y_ENABLE_PIN) || !READ(Z_ENABLE_PIN) || !READ(E_ENABLE_PIN)) //If any of the drivers are enabled...
    {
      lastMotor = millis(); //... set time to NOW so the fan will turn on
    }
    
    if ((millis() - lastMotor) >= (CONTROLLERFAN_SEC*1000UL) || lastMotor == 0) //If the last time any driver was enabled, is longer since than CONTROLLERSEC...
    {
      WRITE(CONTROLLERFAN_PIN, LOW); //... turn the fan off
    }
    else
    {
      WRITE(CONTROLLERFAN_PIN, HIGH); //... turn the fan on
    }
  }
}
#endif

