// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL
#include <WProgram.h>
#include "fastio.h"
extern "C" void __cxa_pure_virtual();
void __cxa_pure_virtual(){};
void get_command();
void process_commands();

void manage_inactivity(byte debug);
void setup_acceleration();

void manage_heater();

#if defined HEATER_USES_THERMISTOR
#define temp2analogh( c ) temp2analog_thermistor(c,temptable,NUMTEMPS)
#define analog2temp( c ) analog2temp_thermistor(c,temptable,NUMTEMPS)
#elif defined HEATER_USES_AD595
#define temp2analogh( c ) temp2analog_ad595(c)
#define analog2temp( c ) analog2temp_ad595(c)
#elif defined HEATER_USES_MAX6675
#define temp2analogh( c ) temp2analog_max6675(c)
#define analog2temp( c ) analog2temp_max6675(c)
#endif

#if defined BED_USES_THERMISTOR
#define temp2analogBed( c ) temp2analog_thermistor((c),bedtemptable,BNUMTEMPS)
#define analog2tempBed( c ) analog2temp_thermistor((c),bedtemptable,BNUMTEMPS)
#elif defined BED_USES_AD595
#define temp2analogBed( c ) temp2analog_ad595(c)
#define analog2tempBed( c ) analog2temp_ad595(c)
#elif defined BED_USES_MAX6675
#define temp2analogBed( c ) temp2analog_max6675(c)
#define analog2tempBed( c ) analog2temp_max6675(c)
#endif

#if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
int temp2analog_thermistor(int celsius, const short table[][2], int numtemps);
int analog2temp_thermistor(int raw,const short table[][2], int numtemps);
#endif

#if defined (HEATER_USES_AD595) || defined (BED_USES_AD595)
int temp2analog_ad595(int celsius);
int analog2temp_ad595(int raw);
#endif

#if defined (HEATER_USES_MAX6675) || defined (BED_USES_MAX6675)
int temp2analog_max6675(int celsius);
int analog2temp_max6675(int raw);
#endif

#if X_ENABLE_PIN > -1
#define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
#define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
#define enable_x() ;
#define disable_x() ;
#endif
#if Y_ENABLE_PIN > -1
#define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
#define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
#define enable_y() ;
#define disable_y() ;
#endif
#if Z_ENABLE_PIN > -1
#define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
#define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
#else
#define enable_z() ;
#define disable_z() ;
#endif
#if E_ENABLE_PIN > -1
#define  enable_e() WRITE(E_ENABLE_PIN, E_ENABLE_ON)
#define disable_e() WRITE(E_ENABLE_PIN,!E_ENABLE_ON)
#else
#define enable_e() ;
#define disable_e() ;
#endif

void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void prepare_move();
void linear_move(unsigned long steps_remaining[]);
void do_step(int axis);
void kill(byte debug);

