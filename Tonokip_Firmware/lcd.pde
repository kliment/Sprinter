#include "lcd.h"
#ifdef FANCY_LCD
#include <LiquidCrystal.h>
#include "pins.h"

LiquidCrystal lcd(LCD_PINS_RS, LCD_PINS_ENABLE, LCD_PINS_D4, LCD_PINS_D5,LCD_PINS_D6,LCD_PINS_D7);  //RS,Enable,D4,D5,D6,D7 

void lcd_status()
{
	
	String line1;
	static char blink=0;
	line1=((blink++)%2==0)? (char)2:' ';
	line1 += int(analog2temp(current_raw));
	line1 += "/";
	line1 += int(analog2temp(target_raw));
	line1 += "\1,";
	line1 += int(analog2tempBed(current_bed_raw));
	line1 +="\1";
	//String line2;

	line1+= (!digitalRead(X_MIN_PIN))? 'x':' ';
	line1+= (!digitalRead(X_MAX_PIN))? 'X':' ';
	line1+= (!digitalRead(Y_MIN_PIN))? 'y':' ';
	line1+= (!digitalRead(Y_MAX_PIN))? 'Y':' ';
	line1+= (!digitalRead(Z_MIN_PIN))? 'z':' ';
	line1+= (!digitalRead(Z_MAX_PIN))? 'Z':' ';
	if(line1.length()>20)
	{
		line1[19]=0;
	}
	lcd.setCursor(0,0); //termometer sign
	lcd.print(line1);
	lcd.setCursor(0, 1); //termometer sign
	
	char cline2[20];
	memcpy(cline2,cmdbuffer[(bufindr-1)%BUFSIZE],19); //the last processed line
	cline2[19]=0;
	bool print=(strlen(cline2)>1);
	bool empty=false;
	for(int i=0;i<19;i++)
	{
 		if(cline2[i]==0) 
			empty=true;
		if(empty)
			cline2[i]=' ';
	}
		
	cline2[19]=0;
	if(1&&print)
	{
		lcd.print(cline2);
		//lcd.print("     ");
	}
}

void lcd_init()
{
	byte Degree[8] =
	{
		B01100,
		B10010,
		B10010,
		B01100,
		B00000,
		B00000,
		B00000,
		B00000
	};
	byte Thermometer[8] =
	{
		B00100,
		B01010,
		B01010,
		B01010,
		B01010,
		B10001,
		B10001,
		B01110
	};

	lcd.begin(16, 2);
	lcd.createChar(1,Degree);
	lcd.createChar(2,Thermometer);
	lcd.clear();
	lcd.print("Sprinter!");
	lcd.setCursor(0, 1);
	lcd.print("Ready to connect");
}


#endif