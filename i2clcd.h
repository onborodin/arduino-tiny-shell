/*

 PIN-Assignment:
 PCF8574	<->		LCD
 ----------------------------------------------
 P0		<->		DB4
 P1		<->		DB5
 P2		<->		DB6
 P3		<->		DB7
 P4		<->		RS
 P5		<->		R/W
 P6		<->		-
 P7		<->		Enable \endverbatim
 
 Example:
 #include "i2clcd.h"

 int main(void) {
	...
	lcd_init();						//-	Display initialization
	...
	char string[] = "Hi World";
	lcd_print(string);					//-	Print a string
	lcd_gotolr(2,4);					//-	Move to position (line 2, row 4)
	
	//-	Turn cursor off and activate blinking
	lcd_command(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGON);
	...
 }
 */

#ifndef _I2CLCD_H
#define _I2CLCD_H

/* SYSTEM CONFIGURATION
    Change this settings to your configuration.
 */

#ifndef F_CPU
#define F_CPU 16000000                          /**< Change this to the clock-rate of your microcontroller */
#endif

//#define wait1us	_delay_loop_1((F_CPU * 0.000001) / 3)   /**< 1 us delay */
//#define wait1ms	_delay_loop_2((F_CPU * 0.001) / 4)      /**< 1 ms delay */
#define wait1us _delay_us(1)    /**< 1 us delay */
#define wait1ms _delay_ms(1)    /**< 1 ms delay */


//-------------------------------------------------------------------------------------------------------------------

#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>

#include <twim.h>

//--Display-Configuration-Settings-----------------------------------------------------------------------------------

/** \defgroup DISPLAY_CONFIGURATION DISPLAY CONFIGURATION
 Change this settings to your configuration.
*/

#ifndef LCD_I2C_DEVICE
#define LCD_I2C_DEVICE	0x38    /* Change this to the address of your expander */
#endif
#define LCD_LINES	2       /* Enter the number of lines of your display here */
#define LCD_ROWS	16      /* Enter the number of rows of your display here */

#define LCD_LINE1	0x00    /* This should be 0x00 on all displays */
#define LCD_LINE2	0x40    /* Change this to the address for line 2 on your display */
#define LCD_LINE3	0x14    /* Change this to the address for line 3 on your display */
#define LCD_LINE4	0x54    /* Change this to the address for line 4 on your display */


//-------------------------------------------------------------------------------------------------------------------

//--The-following-definitions-are-corresponding-to-the-PIN-Assignment-(see-above)------------------------------------

/** \defgroup PIN_ASSIGNMENT PIN ASSIGNMENT
 This pin assignment shows how the display is connected to the PCF8574.
*/

#define LCD_D4_PIN	4       /* LCD-Pin D4 is connected to P4 on the PCF8574 */
#define LCD_D5_PIN	5       /* LCD-Pin D5 is connected to P5 on the PCF8574 */
#define LCD_D6_PIN	6       /* LCD-Pin D6 is connected to P6 on the PCF8574 */
#define LCD_D7_PIN	7       /* LCD-Pin D7 is connected to P7 on the PCF8574 */

#define LCD_RS_PIN	0       /* LCD-Pin RS is connected to P0 on the PCF8574 */
#define LCD_RW_PIN	1       /* LCD-Pin RW is connected to P1 on the PCF8574 */
//#define LCD_EMPTY_PIN                 6       /* this pin is not connected */
#define LCD_BL_PIN	3       /* this pin is not connected */
#define LCD_E_PIN	2       /* LCD-Pin E is connected to P7 on the PCF8574 */


//-------------------------------------------------------------------------------------------------------------------

/** \defgroup DEFINED_BITS DEFINED BITS
 With each read/write operation to/from the display two bytes are send/received. \n
 In each of those two bytes the higher nibble contains the RS, RW, EMPTY and ENABLE bit.
 In the byte which is read/written first, the lower nibble contains bits 0 to 3 and \n 
 in the second byte the lower nibble contains bit 4 to 7. 
*/

#define LCD_D0		(1 << LCD_D4_PIN)       /* bit 0 in 1st lower nibble */
#define LCD_D1		(1 << LCD_D5_PIN)       /* bit 1 in 1st lower nibble */
#define LCD_D2		(1 << LCD_D6_PIN)       /* bit 2 in 1st lower nibble */
#define LCD_D3		(1 << LCD_D7_PIN)       /* bit 3 in 1st lower nibble */

#define LCD_D4		(1 << LCD_D4_PIN)       /* bit 4 in 2nd lower nibble */
#define LCD_D5		(1 << LCD_D5_PIN)       /* bit 5 in 2nd lower nibble */
#define LCD_D6		(1 << LCD_D6_PIN)       /* bit 6 in 2nd lower nibble */
#define LCD_D7		(1 << LCD_D7_PIN)       /* bit 7 in 2nd lower nibble */

#define LCD_RS		(1 << LCD_RS_PIN)       /* RS-bit in 1st and 2nd higher nibble */
#define LCD_RW		(1 << LCD_RW_PIN)       /* RW-bit in 1st and 2nd higher nibble */
//#define LCD_EMPTY                     (1 << LCD_EMPTY_PIN)    /* empty-bit in 1st and 2nd higher nibble */
#define LCD_BL		(1 << LCD_BL_PIN)       /* empty-bit in 1st and 2nd higher nibble */
#define LCD_E		(1 << LCD_E_PIN)        /* E-bit in 1st and 2nd higher nibble */


/* DEFINED READ MODES */

#define LCD_ADDRESS	0       /* Used for reading the address-counter and busy-flag */
#define LCD_DATA	1       /* Used for reading data */


/* LCD-COMMANDS */

/* DEFINED COMMANDS
 These defined commands should be used to configure the display.
 Don't use commands from different categories together.

 Configuration commands from one category should get combined to one command.
 Example:
 lcd_command(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGON);

 The category modes like LCD_SHIFTMODE and LCD_CONFIGURATION can be omitted.
*/

/* GENERAL COMMANDS */

#define LCD_CLEAR	0x01    /* Clear screen */
#define LCD_HOME	0x02    /* Cursor move to first position */
#define LCD_CGRAM	0x06	/* Set CGRAM address */
#define LCD_DDRAM	0x07	/* Set DDRAM address */

/* ENTRYMODES */
#define LCD_ENTRYMODE		0x04                            /* Set entrymode */
#define LCD_INCREASE		LCD_ENTRYMODE | 0x02            /*	Set cursor move direction -- Increase */
#define LCD_DECREASE		LCD_ENTRYMODE | 0x00            /*	Set cursor move direction -- Decrease */
#define LCD_DISPLAYSHIFTON	LCD_ENTRYMODE | 0x01            /*	Display is shifted */
#define LCD_DISPLAYSHIFTOFF	LCD_ENTRYMODE | 0x00            /*	Display is not shifted */

/* DISPLAYMODES */
#define LCD_DISPLAYMODE		0x08                            /* Set displaymode */
#define LCD_DISPLAYON		LCD_DISPLAYMODE | 0x04          /*	Display on */
#define LCD_DISPLAYOFF		LCD_DISPLAYMODE | 0x00          /*	Display off */
#define LCD_CURSORON		LCD_DISPLAYMODE | 0x02          /*	Cursor on */
#define LCD_CURSOROFF		LCD_DISPLAYMODE | 0x00          /*	Cursor off */
#define LCD_BLINKINGON		LCD_DISPLAYMODE | 0x01          /*	Blinking on */
#define LCD_BLINKINGOFF		LCD_DISPLAYMODE | 0x00          /*	Blinking off */

/* SHIFTMODES */
#define LCD_SHIFTMODE		0x10                            /* Set shiftmode */
#define LCD_DISPLAYSHIFT	LCD_SHIFTMODE | 0x08            /*	Display shift */
#define LCD_CURSORMOVE		LCD_SHIFTMODE | 0x00            /*	Cursor move */
#define LCD_RIGHT		LCD_SHIFTMODE | 0x04            /*	Right shift */
#define LCD_LEFT		LCD_SHIFTMODE | 0x00            /*	Left shift */

/* DISPLAY_CONFIGURATION */
#define LCD_CONFIGURATION	0x20                            /* Set function */
#define LCD_8BIT		LCD_CONFIGURATION | 0x10        /*	8 bits interface */
#define LCD_4BIT		LCD_CONFIGURATION | 0x00        /*	4 bits interface */
#define LCD_2LINE		LCD_CONFIGURATION | 0x08        /*	2 line display */
#define LCD_1LINE		LCD_CONFIGURATION | 0x00        /*	1 line display */
#define LCD_5X10		LCD_CONFIGURATION | 0x04        /*	5 X 10 dots */
#define LCD_5X7			LCD_CONFIGURATION | 0x00        /*	5 X 7 dots */

#define ON	0
#define OFF	1


/* FUNCTIONS */

void lcd_init(void);                                            /* Display initialization sequence */
void lcd_write_i2c(uint8_t value);                              /* Write data to i2c */
void lcd_write(uint8_t value);                                  /* Write byte to display with toggle of enable-bit */
bool lcd_gotolr(uint8_t line, uint8_t row);                     /* Go to position */

void lcd_putchar(uint8_t value);                                /* Put char to cursor position */
bool lcd_putcharlr(uint8_t line, uint8_t row, uint8_t value);   /* Put char to position */
void lcd_print(uint8_t *string);                                /* Print string to cursor position */

bool lcd_printlr(uint8_t line, uint8_t row, uint8_t *string);   /* Print string to position */
bool lcd_printlc(uint8_t line, uint8_t row, uint8_t *string);   /* Print string to position, overwrite first chars */
bool lcd_printlrc(uint8_t line, uint8_t row, uint8_t *string);  /* Print string to position, owerwrite next line */

void lcd_command(uint8_t command);                              /* Issue a command to the display */
void lcd_wait_us(uint16_t us);                                  /* Wait some microseconds */
void lcd_wait_ms(uint16_t ms);                                  /* Wait some milliseconds */
void lcd_backlight(int bl);                                     /* ON/OFF backlight bit for next operation*/
void lcd_clear(void);                                           /* Clear screen */

#endif
/* EOF */
