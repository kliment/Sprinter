// Thermistor lookup table for RepRap Temperature Sensor Boards (http://make.rrrf.org/ts)
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=10000 --t0=25 --r1=0 --r2=4700 --beta=3977 --max-adc=1023
// r0: 10000
// t0: 25
// r1: 0
// r2: 4700
// beta: 3977
// max adc: 1023
#define NUMTEMPS 20
short bedtemptable[NUMTEMPS][2] = {
   {1, 430},
   {54, 137},
   {107, 107},
   {160, 91},
   {213, 80},
   {266, 71},
   {319, 64},
   {372, 57},
   {425, 51},
   {478, 46},
   {531, 41},
   {584, 35},
   {637, 30},
   {690, 25},
   {743, 20},
   {796, 14},
   {849, 7},
   {902, 0},
   {955, -11},
   {1008, -35}
};

