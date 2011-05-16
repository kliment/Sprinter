#include "Axis.h"

volatile unsigned long extracalls = 0;

Axis::Axis(int step_pin, int dir_pin, int enable_pin, int min_pin, int max_pin, float steps_per_unit, bool enable_inverted, bool dir_inverted, float max_length)
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

	acceleration_enabled = false;
	accelerating = false;

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
	
	// return time for move
	// return (steps_to_take / (steps_per_unit * feedrate / 60000000));
	return (diff/feedrate) * 60000000;
}

float Axis::set_time_for_move(unsigned long tfm)
{
	if(steps_to_take)
		interval = tfm / steps_to_take;
}

void Axis::precomputemove()
{
	if(direction) digitalWrite(dir_pin, !dir_inverted);
	else digitalWrite(dir_pin, dir_inverted);

	steps_remaining = steps_to_take;
	if(steps_remaining) enable();
	steps_done = 0;
	lastinterval = 0;
}


bool Axis::move(unsigned long micros_now)
{
	if(steps_remaining <= 0)
		return false;

	unsigned long intervalspassed = micros_now / interval;
	if(intervalspassed == lastinterval)
	{
		extracalls++;
		if(steps_remaining) return true;
		else return false;
	}
	//Serial.print("INTS:"); Serial.print(intervalspassed,DEC);
	//Serial.print("SR:"); Serial.print(steps_remaining,DEC);
	//Serial.print("SD:"); Serial.println(steps_done,DEC);

	lastinterval = intervalspassed;
	do_step();
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

	if(destination > current) direction = true;
	else direction = false;

	if(MIN_SOFTWARE_ENDSTOPS) 
		if(destination < 0) destination = 0;
	if(MAX_SOFTWARE_ENDSTOPS)
		if(destination > max_length) destination = max_length;

	extracalls = 0;
}

void Axis::debug()
{
	Serial.print("EC:"); Serial.println(extracalls,DEC);
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

unsigned long Axis::precompute_accel(unsigned long interval,unsigned int delta)
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
#endif

