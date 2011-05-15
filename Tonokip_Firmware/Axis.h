#ifndef __AXIS_H__ 
#define __AXIS_H__

#include <WProgram.h>
#include "configuration.h"

class Axis
{
  public:
	bool direction;
	float destination;
	float current;
	unsigned long interval;
	float feedrate;
	float steps_per_unit;
	float max_length;

	bool relative;
	
	bool acceleration_enabled;
	bool accelerating;

	int step_pin;
	int dir_pin;
	bool dir_inverted;
	int enable_pin;
	bool enable_inverted;
	int min_pin;
	int max_pin;
	
	Axis(int step_pin, int dir_pin, int enable_pin, int min_pin, int max_pin, float steps_per_unit, bool enable_inverted, bool dir_inverted, float max_length);

	void enable();
	void disable();
	void do_step();
	unsigned long get_time_for_move(float feedrate);
	float set_time_for_move(unsigned long tfm);
	void precomputemove();
	bool move(unsigned long micros_now);
	bool is_moving();
	void set_target(float target);
	void debug();

	unsigned long steps_to_take;
	unsigned long steps_remaining;
	unsigned long steps_done;
	unsigned long lastinterval;
};

#endif // __AXIS_H__
