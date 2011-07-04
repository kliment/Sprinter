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

#define BNUMTEMPS 61
const short bedtemptable[BNUMTEMPS][2] = {
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
{	1008	,	0	},

};


#endif

