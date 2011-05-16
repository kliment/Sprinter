#include "Axis.h"

Axis::Axis(int step_pin, int dir_pin, int enable_pin, int min_pin, int max_pin, 
           float steps_per_unit, bool enable_inverted, bool dir_inverted, float max_length, float max_feedrate,int homing_dir)
{
	// Initialize class data
	direction = false;
	destination = 0;
	current = 0;
	interval = 0;
	feedrate = 0;
	relative = false;
	this->steps_per_unit = steps_per_unit;
	this->enable_inverted = enable_inverted;
	this->dir_inverted   = dir_inverted;
	this->max_length = max_length;
  this->max_feedrate = max_feedrate;
  this->homing_dir = homing_dir;

	acceleration_enabled = false;
	accelerating = false;
  decelerating = false;

	this->step_pin 	= step_pin;
	this->dir_pin 	= dir_pin;
	this->enable_pin = enable_pin;
	this->min_pin 	= min_pin;
	this->max_pin 	= max_pin;

	// Initialize pins we control.
	if(step_pin > -1) { pinMode(step_pin, OUTPUT); digitalWrite(step_pin, LOW); }
	if(dir_pin > -1) { pinMode(dir_pin, OUTPUT); digitalWrite(dir_pin, LOW); }
	if(enable_pin > -1) { pinMode(enable_pin, OUTPUT); digitalWrite(enable_pin, LOW); }
	if(min_pin > -1) 
	{
		pinMode(min_pin, INPUT); 
		digitalWrite(min_pin, ENDSTOPPULLUPS);
	}
	if(max_pin > -1)
	{
		pinMode(max_pin, INPUT);
		digitalWrite(max_pin, ENDSTOPPULLUPS);
	}
}
	
void Axis::home()
{
  current = 0;
  if(homing_dir == 0)
    return;
    
  destination = 1.5 * max_length * homing_dir;
  Serial.print("Homing: ");Serial.println(destination);
  prepare_move(); // Strange way to move - call global prepare_move on all steppers.
  current = 0; 
  destination = 0;
}


void Axis::enable()
{
	if(enable_pin > -1) digitalWrite(enable_pin, !enable_inverted);
}

void Axis::disable()
{
	if(enable_pin > -1) digitalWrite(enable_pin, enable_inverted);
}

void Axis::do_step()
{
  if(steps_remaining == 0) return;
	if(direction)
	{
		if(max_pin > -1 && digitalRead(max_pin) != ENDSTOPS_INVERTING) 
		{
			steps_remaining = 0;
			return;
		}
		current += 1 / steps_per_unit;
	}
	else
	{
		if(min_pin > -1 && digitalRead(min_pin) != ENDSTOPS_INVERTING) 
		{
			steps_remaining = 0;
			return;
		}
		current -= 1 / steps_per_unit;
	}
	steps_remaining--;
	steps_done++;

	digitalWrite(step_pin, HIGH);
	digitalWrite(step_pin, LOW);
}

unsigned long Axis::get_time_for_move(float feedrate)
{
	if(feedrate > max_feedrate) feedrate = max_feedrate;
	if(feedrate < MINIMUM_FEEDRATE) feedrate = MINIMUM_FEEDRATE;

	float diff = abs(destination - current);
	steps_to_take = diff * steps_per_unit;
  unsigned long tfm = (steps_to_take / (steps_per_unit * feedrate / 60000000));

  Serial.print("feed: ");Serial.print(feedrate);
  Serial.print(" tfm: ");Serial.print(tfm);
  Serial.print(" steps: ");Serial.println(steps_to_take);

	
	// return time for move
	return tfm;
	// return (diff/feedrate) * 60000000;
}

float Axis::set_time_for_move(unsigned long tfm)
{
	if(steps_to_take)
		interval = (tfm * 100) / steps_to_take;  // interval mesaured in nanos
  else
    interval = 0;

  Serial.print("interval: ");Serial.println(interval);
}

void Axis::precomputemove()
{
	if(destination > current) direction = true;
	else direction = false;

	if(direction) digitalWrite(dir_pin, !dir_inverted);
	else digitalWrite(dir_pin, dir_inverted);

	steps_remaining = steps_to_take;
	if(steps_remaining) enable();
	steps_done = 0;
}

bool Axis::is_moving()
{
	if(steps_remaining > 0) return true;
	return false;
}

void Axis::set_target(float target)
{
	if(relative)
	  destination += target;
	else
	  destination = target;

	if(MIN_SOFTWARE_ENDSTOPS) 
		if(destination < 0) destination = 0;
	if(MAX_SOFTWARE_ENDSTOPS)
		if(destination > max_length) destination = max_length;

}

#ifdef EXP_ACCELERATION
void Axis::setup_accel()
{
  // Eventually these /could/ be different per axis.
  full_velocity_units = FULL_VELOCITY_UNITS; 
  min_units_per_second = MIN_UNITS_PER_SECOND;
  min_constant_speed_units = MIN_CONSTANT_SPEED_UNITS;

  long_full_velocity_units = full_velocity_units * 100;
  max_interval = 100000000.0 / (min_units_per_second * steps_per_unit);
  min_constant_speed_steps = min_constant_speed_units * steps_per_unit;
  virtual_full_velocity_steps = long_full_velocity_units * steps_per_unit /100;
  full_velocity_steps = 0;
  full_interval = 0;
  steps_acceleration_check = 1;
}

void Axis::precompute_accel(unsigned long interval,unsigned int delta)
{
  full_velocity_steps = min(virtual_full_velocity_steps, (delta - min_constant_speed_steps) / 2);
  acceleration_enabled = true;

  if(full_velocity_steps == 0) full_velocity_steps++;
  if(interval > max_interval) acceleration_enabled = false;

  full_interval = interval;

  if(min_constant_speed_steps >= steps_to_take) 
  {
    acceleration_enabled = false;
    full_interval = max(max_interval, interval); // choose the min speed between feedrate and acceleration start speed
  }

  if(full_velocity_steps < virtual_full_velocity_steps && acceleration_enabled) 
  {
    // choose the min speed between feedrate and speed at full steps
    full_interval = max(interval, max_interval - ((max_interval - full_interval) * full_velocity_steps / virtual_full_velocity_steps));
  }
  accelerating = acceleration_enabled;
}

unsigned long Axis::recompute_accel(unsigned long timediff, unsigned long interval)
{
    //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
    if (acceleration_enabled && steps_done < full_velocity_steps && steps_done / full_velocity_steps < 1 && (steps_done % steps_acceleration_check == 0)) 
    {
      if(steps_done == 0)
        interval = max_interval;
      else 
        interval = max_interval - ((max_interval - full_interval) * steps_done / virtual_full_velocity_steps);
    } 
    else if (acceleration_enabled && steps_remaining < full_velocity_steps) 
    {
      //Else, if acceleration is enabled on this move and we are in the deceleration segment, calculate the current interval
      if(steps_remaining == 0) 
        interval = max_interval;
      else
        interval = max_interval - ((max_interval - full_interval) * steps_remaining / virtual_full_velocity_steps);
      accelerating = true;
    } 
    else if (steps_done - full_velocity_steps >= 1 || !acceleration_enabled)
    {
      //Else, we are just use the full speed interval as current interval
      interval = full_interval;
      accelerating = false;
    }
    return interval;
}
#endif // EXP_ACCELERATION

#ifdef RAMP_ACCELERATION
void Axis::setup_accel()
{
  max_interval = 100000000.0 / (MIN_UNITS_PER_SECOND * steps_per_unit);
  steps_per_sqr_second = MAX_ACCELERATION_UNITS_PER_SQ_SECOND * steps_per_unit;
  plateau_steps = 0;
  plateau_time = 0;
  max_speed_steps_per_second = 0;
  min_speed_steps_per_second = 0;
  full_interval = 0;
  start_move_micros = 0;
}

void Axis::precompute_accel(unsigned long interval,unsigned int delta)
{
  max_speed_steps_per_second = 100000000 / interval;
  min_speed_steps_per_second = 100000000 / max_interval;
  plateau_time = (max_speed_steps_per_second - min_speed_steps_per_second) / (float) steps_per_sqr_second;
  plateau_steps = (long) ((steps_per_sqr_second / 2.0 * plateau_time + min_speed_steps_per_second) * plateau_time);
  plateau_steps *= 1.01; // This is to compensate we use discrete intervals
  acceleration_enabled = true;
  full_interval = interval;
  if(interval > max_interval) acceleration_enabled = false;
  decelerating = false;
  start_move_micros = micros();
}

unsigned long Axis::recompute_accel(unsigned long timediff, unsigned long interval)
{
  //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
  if (acceleration_enabled && steps_done == 0) 
  {
    interval = max_interval;
  } 
  else if (acceleration_enabled && steps_done <= plateau_steps) 
  {
    long current_speed = (long) ((((long) steps_per_sqr_second) / 10000) * ((micros() - start_move_micros)  / 100) + (long) min_speed_steps_per_second);
    interval = 100000000 / current_speed;
    if (interval < full_interval) 
    {
      accelerating = false;
      interval = full_interval;
    }
    if (steps_done >= steps_to_take / 2) 
    {
      plateau_steps = steps_done;
      max_speed_steps_per_second = 100000000 / interval;
      accelerating = false;
    }
  } 
  else if (acceleration_enabled && steps_remaining <= plateau_steps) 
  { 
    if (!accelerating) 
    {
      // Huh?  why do we change this here?
      start_move_micros = micros();
      accelerating = true;
      decelerating = true;
    }
    long current_speed = (long) ((long) max_speed_steps_per_second - ((((long) steps_per_sqr_second) / 10000) * ((micros() - start_move_micros) / 100)));
    interval = 100000000 / current_speed;
    if (interval > max_interval)
      interval = max_interval;
  } 
  else 
  {
    //Else, we are just use the full speed interval as current interval
    interval = full_interval;
    accelerating = false;
  }
  return interval;
}
#endif // RAMP_ACCELERATION
