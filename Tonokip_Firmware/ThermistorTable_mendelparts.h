#ifndef THERMISTORTABLE_H_
#define THERMISTORTABLE_H_

//thermistor table for mendel-parts thermistor

#define NUMTEMPS 20
short temptable[NUMTEMPS][2] = {
   {1, 827},
   {54, 253},
   {107, 207},
   {160, 182},
   {213, 165},
   {266, 152},
   {319, 141},
   {372, 132},
   {425, 123},
   {478, 115},
   {531, 107},
   {584, 100},
   {637, 93},
   {690, 86},
   {743, 78},
   {796, 70},
   {849, 61},
   {902, 49},
   {955, 34},
   {1008, 3}
};


#endif

