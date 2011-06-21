#ifndef __LCDH
#define __LCDH
#include <WProgram.h>
#include "configuration.h"

#ifdef FANCY_LCD
void lcd_status();
void lcd_init();
#else

//empty inline if lcd is disabled
inline void lcd_status(){	}
inline void lcd_init(){	}

#endif


#endif