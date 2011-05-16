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
#ifdef EXP_ACCELERATION
  void setup_accel();
  unsigned long precompute_accel(unsigned long interval,unsigned int delta);
  unsigned long recompute_accel(unsigned long timediff, unsigned long interval);
  float full_velocity_units;
  float min_units_per_second;
  float min_constant_speed_units;
  unsigned long long_full_velocity_units;
  unsigned long max_interval;
  unsigned long min_constant_speed_steps;
  unsigned long virtual_full_velocity_steps;
  unsigned long full_velocity_steps;
  unsigned long full_interval;
  unsigned int steps_acceleration_check;
#else
  void setup_accel() { return; };
  unsigned long precompute_accel(unsigned long interval,unsigned int delta) { return interval };
  unsigned long recompute_accel(unsigned long timediff, unsigned long interval) { return interval };
#endif

  
};

#endif // __AXIS_H__
