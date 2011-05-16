// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL
#include <WProgram.h>

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
void linear_move();
bool axis_are_moving();
bool non_accel_axis_are_moving();

void kill(byte debug);


// exported globals
// feedrate should probably be handled differently
extern float feedrate;
