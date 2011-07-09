// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL
#include <WProgram.h>
extern "C" void __cxa_pure_virtual();
void __cxa_pure_virtual(){};
void get_command();
void process_commands();

void manage_inactivity(byte debug);

void manage_heater();
float temp2analog(int celsius);
float temp2analogBed(int celsius);
float analog2temp(int raw);
float analog2tempBed(int raw);

void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void prepare_move();
void linear_move(unsigned long steps_remaining[]);
void do_step_update_micros(int axis);
void disable_x();
void disable_y();
void disable_z();
void disable_e();
void enable_x();
void enable_y();
void enable_z();
void enable_e();
void do_step(int axis);

void kill(byte debug);

