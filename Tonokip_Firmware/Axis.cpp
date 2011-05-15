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

#ifdef EXP_ACCELERATION
	max_interval = 0;
	steps_per_sqr_second = 0;
	travel_steps_per_sqr_second = 0;
#endif
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

inline void Axis::do_step()
{
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
	if(direction)
	{
		if(max_pin > -1 && digitalRead(max_pin) != ENDSTOPS_INVERTING) steps_remaining = 0;
	}
	else
	{
		if(min_pin > -1 && digitalRead(min_pin) != ENDSTOPS_INVERTING) steps_remaining = 0;
	}

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
	steps_remaining--;
	steps_done++;
	do_step();
	if(direction)
		current += 1 / steps_per_unit;
	else
		current -= 1 / steps_per_unit;
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

