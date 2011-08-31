#ifndef THERMISTORTABLES_H_
#define THERMISTORTABLES_H_

#if (THERMISTORHEATER == 1) || (THERMISTORBED == 1) //100k bed thermistor


#define NUMTEMPS_1 61
const short temptable_1[NUMTEMPS_1][2] = {
{	23	,	300	},
{	25	,	295	},
{	27	,	290	},
{	28	,	285	},
{	31	,	280	},
{	33	,	275	},
{	35	,	270	},
{	38	,	265	},
{	41	,	260	},
{	44	,	255	},
{	48	,	250	},
{	52	,	245	},
{	56	,	240	},
{	61	,	235	},
{	66	,	230	},
{	71	,	225	},
{	78	,	220	},
{	84	,	215	},
{	92	,	210	},
{	100	,	205	},
{	109	,	200	},
{	120	,	195	},
{	131	,	190	},
{	143	,	185	},
{	156	,	180	},
{	171	,	175	},
{	187	,	170	},
{	205	,	165	},
{	224	,	160	},
{	245	,	155	},
{	268	,	150	},
{	293	,	145	},
{	320	,	140	},
{	348	,	135	},
{	379	,	130	},
{	411	,	125	},
{	445	,	120	},
{	480	,	115	},
{	516	,	110	},
{	553	,	105	},
{	591	,	100	},
{	628	,	95	},
{	665	,	90	},
{	702	,	85	},
{	737	,	80	},
{	770	,	75	},
{	801	,	70	},
{	830	,	65	},
{	857	,	60	},
{	881	,	55	},
{	903	,	50	},
{	922	,	45	},
{	939	,	40	},
{	954	,	35	},
{	966	,	30	},
{	977	,	25	},
{	985	,	20	},
{	993	,	15	},
{	999	,	10	},
{	1004	,	5	},
{	1008	,	0	} //safety
};
#endif
#if (THERMISTORHEATER == 2) || (THERMISTORBED == 2) //200k bed thermistor
#define NUMTEMPS_2 21
const short temptable_2[NUMTEMPS_2][2] = {
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
   {1008, 17},
   {1020, 0} //safety
};

#endif
#if (THERMISTORHEATER == 3) || (THERMISTORBED == 3) //mendel-parts
#define NUMTEMPS_3 28
const short temptable_3[NUMTEMPS_3][2] = {
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
#if (THERMISTORHEATER == 4) || (THERMISTORBED == 4) //10k thermistor

#define NUMTEMPS_4 20
short temptable_4[NUMTEMPS_4][2] = {
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
#endif

#if (THERMISTORHEATER == 5) || (THERMISTORBED == 5) //100k ParCan thermistor (104GT-2)

#define NUMTEMPS_5 61
const short temptable_5[NUMTEMPS_5][2] = {
{1, 713},
{18, 316},
{35, 266},
{52, 239},
{69, 221},
{86, 208},
{103, 197},
{120, 188},
{137, 181},
{154, 174},
{171, 169},
{188, 163},
{205, 159},
{222, 154},
{239, 150},
{256, 147},
{273, 143},
{290, 140},
{307, 136},
{324, 133},
{341, 130},
{358, 128},
{375, 125},
{392, 122},
{409, 120},
{426, 117},
{443, 115},
{460, 112},
{477, 110},
{494, 108},
{511, 106},
{528, 103},
{545, 101},
{562, 99},
{579, 97},
{596, 95},
{613, 92},
{630, 90},
{647, 88},
{664, 86},
{681, 84},
{698, 81},
{715, 79},
{732, 77},
{749, 75},
{766, 72},
{783, 70},
{800, 67},
{817, 64},
{834, 61},
{851, 58},
{868, 55},
{885, 52},
{902, 48},
{919, 44},
{936, 40},
{953, 34},
{970, 28},
{987, 20},
{1004, 8},
{1021, 0}
};
#endif

#if (THERMISTORHEATER == 6) || (THERMISTORBED == 6) // 100k Epcos thermistor
#define NUMTEMPS_6 36
const short temptable_6[NUMTEMPS_6][2] = {
   {28, 250},
   {31, 245},
   {35, 240},
   {39, 235},
   {42, 230},
   {44, 225},
   {49, 220},
   {53, 215},
   {62, 210},
   {73, 205},
   {72, 200},
   {94, 190},
   {102, 185},
   {116, 170},
   {143, 160},
   {183, 150},
   {223, 140},
   {270, 130},
   {318, 120},
   {383, 110},
   {413, 105},
   {439, 100},
   {484, 95},
   {513, 90},
   {607, 80},
   {664, 70},
   {781, 60},
   {810, 55},
   {849, 50},
   {914, 45},
   {914, 40},
   {935, 35},
   {954, 30},
   {970, 25},
   {978, 22},
   {1008, 3}
};
#endif

#if THERMISTORHEATER == 1
#define NUMTEMPS NUMTEMPS_1
#define temptable temptable_1
#elif THERMISTORHEATER == 2
#define NUMTEMPS NUMTEMPS_2
#define temptable temptable_2
#elif THERMISTORHEATER == 3
#define NUMTEMPS NUMTEMPS_3
#define temptable temptable_3
#elif THERMISTORHEATER == 4
#define NUMTEMPS NUMTEMPS_4
#define temptable temptable_4
#elif THERMISTORHEATER == 5
#define NUMTEMPS NUMTEMPS_5
#define temptable temptable_5
#elif THERMISTORHEATER == 6
#define NUMTEMPS NUMTEMPS_6
#define temptable temptable_6
#elif defined HEATER_USES_THERMISTOR
#error No heater thermistor table specified
#endif
#if THERMISTORBED == 1
#define BNUMTEMPS NUMTEMPS_1
#define bedtemptable temptable_1
#elif THERMISTORBED == 2
#define BNUMTEMPS NUMTEMPS_2
#define bedtemptable temptable_2
#elif THERMISTORBED == 3
#define BNUMTEMPS NUMTEMPS_3
#define bedtemptable temptable_3
#elif THERMISTORBED == 4
#define BNUMTEMPS NUMTEMPS_4
#define bedtemptable temptable_4
#elif THERMISTORBED == 5
#define BNUMTEMPS NUMTEMPS_5
#define bedtemptable temptable_5
#elif THERMISTORBED == 6
#define BNUMTEMPS NUMTEMPS_6
#define bedtemptable temptable_6
#elif defined BED_USES_THERMISTOR
#error No bed thermistor table specified
#endif

#endif //THERMISTORTABLES_H_
