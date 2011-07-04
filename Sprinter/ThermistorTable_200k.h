#ifndef THERMISTORTABLE_H_
#define THERMISTORTABLE_H_

// Thermistor lookup table for RepRap Temperature Sensor Boards (http://make.rrrf.org/ts)
// See this page:  
// http://dev.www.reprap.org/bin/view/Main/Thermistor
// for details of what goes in this table.
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4700 --beta=4066 --max-adc=1023
// r0: 100000
// t0: 25
// r1: 0
// r2: 4700
// beta: 4066
// max adc: 1023

#define NUMTEMPS 20
const short temptable[NUMTEMPS][2] = {
   {1, 848},
   {54, 275},
   {107, 228},
   {160, 202},
   {213, 185},
   {266, 171},
   {319, 160},
   {372, 150},
   {425, 141},
   {478, 133},
   {531, 125},
   {584, 118},
   {637, 110},
   {690, 103},
   {743, 95},
   {796, 86},
   {849, 77},
   {902, 65},
   {955, 49},
   {1008, 17}
};


#endif
