#ifndef THERMISTORTABLE_H_
#define THERMISTORTABLE_H_

//thermistor table for mendel-parts thermistor
// Standardized R/T characteristic no. 8404
    // RS thermistor 484-0183; EPCOS NTC
    // Mendel-Parts thermistor G540 / G550
    // Optimized for 100...300C working range.
    // Max range: -20...300C
    // Max reading error on Gen 6 electronics: ~+5%, -3% in 100 - 300C range.
	
#define NUMTEMPS 28
const short temptable[NUMTEMPS][2] = {
		{1,864},
		{21,300},
		{25,290},
		{29,280},
		{33,270},
		{39,260},
		{46,250},
		{54,240},
		{64,230},
		{75,220},
		{90,210},
		{107,200},
		{128,190},
		{154,180},
		{184,170},
		{221,160},
		{265,150},
		{316,140},
		{375,130},
		{441,120},
		{513,110},
		{588,100},
		{734,80},
		{856,60},
		{938,40},
		{986,20},
		{1008,0},
		{1018,-20}
	};


#endif
