// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL
#include <WProgram.h>

void get_command();
void process_commands();

void manage_inactivity(byte debug);

void manage_heater();
int temp2analogu(int celsius, const short table[][2], int numtemps, int source);
int analog2tempu(int raw, const short table[][2], int numtemps, int source);
#ifdef HEATER_USES_THERMISTOR
    #define HEATERSOURCE 1
#endif
#ifdef HEATER_USES_AD595
    #define HEATERSOURCE 2
#endif
#ifdef HEATER_USES_MAX6675
    #define HEATERSOURCE 3
#endif
#ifdef BED_USES_THERMISTOR
    #define BEDSOURCE 1
#endif
#ifdef BED_USES_AD595
    #define BEDSOURCE 2
#endif
#ifdef BED_USES_MAX6675
    #define BEDSOURCE 3
#endif

#define temp2analogh( c ) temp2analogu((c),temptable,NUMTEMPS,HEATERSOURCE)
#define temp2analogBed( c ) temp2analogu((c),bedtemptable,BNUMTEMPS,BEDSOURCE)
#define analog2temp( c ) analog2tempu((c),temptable,NUMTEMPS,HEATERSOURCE)
#define analog2tempBed( c ) analog2tempu((c),bedtemptable,BNUMTEMPS,BEDSOURCE)

void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void prepare_move();
void linear_move(unsigned long x_steps_remaining, unsigned long y_steps_remaining, unsigned long z_steps_remaining, unsigned long e_steps_remaining);
void disable_x();
void disable_y();
void disable_z();
void disable_e();
void enable_x();
void enable_y();
void enable_z();
void enable_e();
void do_x_step();
void do_y_step();
void do_z_step();
void do_e_step();

void kill(byte debug);

