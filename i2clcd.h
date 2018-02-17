/*****************************************************************************
 
 i2clcd.h - LCD over I2C library 
		Designed for HD44870 based LCDs with I2C expander PCF8574X
		on Atmels AVR MCUs
 
 Copyright (C) 2006 Nico Eichelmann and Thomas Eichelmann
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 
 You can contact the authors at info@computerheld.de
 
*****************************************************************************/

/**
\mainpage

 \par i2clcd.h - LCD over I2C library
	Designed for HD44870 based LCDs with I2C expander PCF8574X
	on Atmels AVR MCUs

 \author Nico Eichelmann, Thomas Eichelmann

 \version 0.2
 
 \par License:
 \subpage LICENSE "GNU Lesser General Public License"
 
 \par Files:
	\subpage I2CLCD.H \n
	\subpage I2CLCD.C

 \note Requires I2C-Library from Peter Fleury http://jump.to/fleury

 \par Original wrote with the following configuration:
	2x16 Display (KS0070B), PCF8574P, ATMega32 @ 16 Mhz \n
	avr-gcc (GCC) 4.1.0 \n

 \par Changed the following configuration:
	2x16 Display (JND1602C), PCA8574AD, ATMega328p @ 16 Mhz \n
	avr-gcc (GCC) 5.4.x \n

 \par PIN-Assignment:
 \verbatim
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
 
 \par Example:
 \code
 #include "i2clcd.h"
 	
 int main(void)
 {
	...
	lcd_init();						//-	Display initialization
	...
	char string[] = "Hi World";
	lcd_print(string);					//-	Print a string
	lcd_gotolr(2,4);					//-	Move to position (line 2, row 4)
	
	//-	Turn cursor off and activate blinking
	lcd_command(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGON);
	...
 } \endcode
 \page LICENSE GNU Lesser General Public License
 \include ./lgpl.txt
 \page I2CLCD.H i2clcd.h
 \include ./i2clcd.h
 \page I2CLCD.C i2clcd.c
 \include ./i2clcd.c
*/

#ifndef _I2CLCD_H
#define _I2CLCD_H

//--System-Configuration-Settings------------------------------------------------------------------------------------

/** \defgroup SYSTEM_CONFIGURATION SYSTEM CONFIGURATION
 Change this settings to your configuration.
*/
/*@{*/
#ifndef F_CPU
#define F_CPU 16000000                          /**< Change this to the clock-rate of your microcontroller */
#endif

//#define wait1us	_delay_loop_1((F_CPU * 0.000001) / 3)   /**< 1 us delay */
//#define wait1ms	_delay_loop_2((F_CPU * 0.001) / 4)      /**< 1 ms delay */
#define wait1us _delay_us(1)    /**< 1 us delay */
#define wait1ms _delay_ms(1)    /**< 1 ms delay */
/*@}*/

//-------------------------------------------------------------------------------------------------------------------

#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>

#include <twim.h>

//--Display-Configuration-Settings-----------------------------------------------------------------------------------

/** \defgroup DISPLAY_CONFIGURATION DISPLAY CONFIGURATION
 Change this settings to your configuration.
*/
/*@{*/
#ifndef LCD_I2C_DEVICE
#define LCD_I2C_DEVICE	0x38    /* Change this to the address of your expander */
#endif
#define LCD_LINES	2       /* Enter the number of lines of your display here */
#define LCD_ROWS	16      /* Enter the number of rows of your display here */

#define LCD_LINE1	0x00    /* This should be 0x00 on all displays */
#define LCD_LINE2	0x40    /* Change this to the address for line 2 on your display */
#define LCD_LINE3	0x10    /* Change this to the address for line 3 on your display */
#define LCD_LINE4	0x50    /* Change this to the address for line 4 on your display */
/*@}*/

//-------------------------------------------------------------------------------------------------------------------

//--The-following-definitions-are-corresponding-to-the-PIN-Assignment-(see-above)------------------------------------

/** \defgroup PIN_ASSIGNMENT PIN ASSIGNMENT
 This pin assignment shows how the display is connected to the PCF8574.
*/
/*@{*/
#define LCD_D4_PIN	4       /* LCD-Pin D4 is connected to P4 on the PCF8574 */
#define LCD_D5_PIN	5       /* LCD-Pin D5 is connected to P5 on the PCF8574 */
#define LCD_D6_PIN	6       /* LCD-Pin D6 is connected to P6 on the PCF8574 */
#define LCD_D7_PIN	7       /* LCD-Pin D7 is connected to P7 on the PCF8574 */

#define LCD_RS_PIN	0       /* LCD-Pin RS is connected to P0 on the PCF8574 */
#define LCD_RW_PIN	1       /* LCD-Pin RW is connected to P1 on the PCF8574 */
//#define LCD_EMPTY_PIN                 6       /* this pin is not connected */
#define LCD_BL_PIN	3       /* this pin is not connected */
#define LCD_E_PIN	2       /* LCD-Pin E is connected to P7 on the PCF8574 */
/*@}*/

//-------------------------------------------------------------------------------------------------------------------

/** \defgroup DEFINED_BITS DEFINED BITS
 With each read/write operation to/from the display two bytes are send/received. \n
 In each of those two bytes the higher nibble contains the RS, RW, EMPTY and ENABLE bit.
 In the byte which is read/written first, the lower nibble contains bits 0 to 3 and \n 
 in the second byte the lower nibble contains bit 4 to 7. 
*/
/*@{*/
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
/*@}*/

/** \defgroup DEFINED_READ_MODES DEFINED READ MODES
*/
/*@{*/
#define LCD_ADDRESS	0       /* Used for reading the address-counter and busy-flag */
#define LCD_DATA	1       /* Used for reading data */
/*@}*/

//-LCD-COMMANDS------------------------------------------------------------------------------------------------------
/** \defgroup DEFINED_COMMANDS DEFINED COMMANDS
 These defined commands should be used to configure the display. \n
 Don't use commands from different categories together. \n
 
 Configuration commands from one category should get combined to one command.
 \par Example: 
 \code lcd_command(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGON); \endcode
 
 The category modes like LCD_SHIFTMODE and LCD_CONFIGURATION can be omitted.
*/
/*@{*/

/** @name GENERAL COMMANDS */
/*@{*/
#define LCD_CLEAR	0x01    /* Clear screen */
#define LCD_HOME	0x02    /* Cursor move to first digit */
/*@}*/

/** @name ENTRYMODES */
/*@{*/
#define LCD_ENTRYMODE		0x04                            /* Set entrymode */
#define LCD_INCREASE		LCD_ENTRYMODE | 0x02            /*	Set cursor move direction -- Increase */
#define LCD_DECREASE		LCD_ENTRYMODE | 0x00            /*	Set cursor move direction -- Decrease */
#define LCD_DISPLAYSHIFTON	LCD_ENTRYMODE | 0x01            /*	Display is shifted */
#define LCD_DISPLAYSHIFTOFF	LCD_ENTRYMODE | 0x00            /*	Display is not shifted */
/*@}*/

/** @name DISPLAYMODES */
/*@{*/
#define LCD_DISPLAYMODE		0x08                            /* Set displaymode */
#define LCD_DISPLAYON		LCD_DISPLAYMODE | 0x04          /*	Display on */
#define LCD_DISPLAYOFF		LCD_DISPLAYMODE | 0x00          /*	Display off */
#define LCD_CURSORON		LCD_DISPLAYMODE | 0x02          /*	Cursor on */
#define LCD_CURSOROFF		LCD_DISPLAYMODE | 0x00          /*	Cursor off */
#define LCD_BLINKINGON		LCD_DISPLAYMODE | 0x01          /*	Blinking on */
#define LCD_BLINKINGOFF		LCD_DISPLAYMODE | 0x00          /*	Blinking off */
/*@}*/

/** @name SHIFTMODES */
/*@{*/
#define LCD_SHIFTMODE		0x10                            /* Set shiftmode */
#define LCD_DISPLAYSHIFT	LCD_SHIFTMODE | 0x08            /*	Display shift */
#define LCD_CURSORMOVE		LCD_SHIFTMODE | 0x00            /*	Cursor move */
#define LCD_RIGHT		LCD_SHIFTMODE | 0x04            /*	Right shift */
#define LCD_LEFT		LCD_SHIFTMODE | 0x00            /*	Left shift */
/*@}*/

/** @name DISPLAY_CONFIGURATION */
/*@{*/
#define LCD_CONFIGURATION	0x20                            /* Set function */
#define LCD_8BIT		LCD_CONFIGURATION | 0x10        /*	8 bits interface */
#define LCD_4BIT		LCD_CONFIGURATION | 0x00        /*	4 bits interface */
#define LCD_2LINE		LCD_CONFIGURATION | 0x08        /*	2 line display */
#define LCD_1LINE		LCD_CONFIGURATION | 0x00        /*	1 line display */
#define LCD_5X10		LCD_CONFIGURATION | 0x04        /*	5 X 10 dots */
#define LCD_5X7			LCD_CONFIGURATION | 0x00        /*	5 X 7 dots */
/*@}*/

#define ON	0
#define OFF	1

//-------------------------------------------------------------------------------------------------------------------

/*@}*/

//-FUNCTIONS---------------------------------------------------------------------------------------------------------

/** \defgroup FUNCTIONS FUNCTIONS */
/*@{*/
/**
 \brief Display initialization sequence
 \return none
 */
void lcd_init(void);            //-     Display initialization sequence

/**
 \brief Write data to i2c (for internal use)
 \param value byte to send over i2c
 \return none
 */
void lcd_write_i2c(uint8_t value);        //-     Write data to i2c

/**
 \brief Write byte to display with toggle of enable-bit
 \param value the upper nibble represents  E, RS, RW pins and the lower nibble contains data D0 to D3 pins or D4 to D7 pins
 \return none
 */
void lcd_write(uint8_t value);    //-     Write byte to display with toggle of enable-bit

/**
 \brief Go to position
 \param line 1st line is 1 and last line = LCD_LINES
 \param row 1st row is 1 and last row = LCD_ROWS
 \retval true if successfull
 \retval false if not successfull
 */
bool lcd_gotolr(uint8_t line, uint8_t row); //-     Go to position

/**
 \brief Put char to cursor position
 \param value the char to print
 \return none
 */
void lcd_putchar(uint8_t value);  //-     Put char to cursor position

/**
 \brief Put char to position
 \param line the line to put the char to
 \param row the row to put the char to
 \param value the char to print
 \retval true if successfull
 \retval false if not successfull
 */
bool lcd_putcharlr(uint8_t line, uint8_t row, uint8_t value); //-     Put char to position

/**
 \brief Print string to cursor position
 \param *string pointer to the string to print
 \return none
 */
void lcd_print(uint8_t *string);  //-     Print string to cursor position

/**
 \brief Print string to position
 \param line the line to put the string to
 \param row the row to put the string to
 \param *string pointer to the string to print
 \retval true if successfull
 \retval false if not successfull
 */
bool lcd_printlr(uint8_t line, uint8_t row, uint8_t *string); //-     Print string to position

/**
 \brief Print string to position (If string is longer than LCD_ROWS overwrite first chars)
 \param line the line to put the string to
 \param row the row to put the string to
 \param *string pointer to the string to print
 \retval true if successfull
 \retval false if not successfull
 */
bool lcd_printlc(uint8_t line, uint8_t row, uint8_t *string);   //-     Print string to position 
                                                                //-     (If string is longer than LCD_ROWS
                                                                //-     overwrite first chars)

/**
 \brief Print string to position (If string is longer than LCD_ROWS continue in next line)
 \param line the line to put the string to
 \param row the row to put the string to
 \param *string pointer to the string to print
 \retval true if successfull
 \retval false if not successfull
 */
bool lcd_printlrc(uint8_t line, uint8_t row, uint8_t *string);      //-     Print string to position 
                                                                    //-     (If string is longer than LCD_ROWS
                                                                    //-     continue in next line)

/**
 \brief Issue a command to the display
 \param command use the defined commands above
 \return none
 */
void lcd_command(uint8_t command);        //-     Issue a command to the display (use the defined commands above)

/**
 \brief Go to nextline (if next line > LCD_LINES return false)
 \retval true if successfull
 \retval false if not successfull
 */
bool lcd_nextline(void);        //-     Go to nextline (if next line > LCD_LINES return false)

/**
 \brief Read data from i2c (for internal use)
 \retval "uint8_t" byte received over i2c
 */
uint8_t lcd_read_i2c(void);       //-     Read data from i2c

/**
 \brief Read data from display over i2c (for internal use)
 \param mode ADDRESS for cursor address and busy flag or DATA for display data
 \retval "uint8_t" the upper nibble represents  E, RS, RW pins and the lower nibble contains data D0 to D3 pins or D4 to D7 pins
 */
uint8_t lcd_read(bool mode);      //-     Read data from display over i2c
                                                        //-     (second nibble corresponds to highbyte or lowbyte)

/**
 \brief Read one byte over i2c from display
 \param mode ADDRESS for cursor address and busy flag or DATA for display data
 \retval "uint8_t" the byte received from the display
 */
uint8_t lcd_getbyte(bool mode);   //-     Read one complete byte over i2c from display

/**
 \brief Get line and row of the cursor position
 \param *line pointer to the target byte for line
 \param *row pointer to the target byte for row
 \retval true if successfull
 \retval false if not successfull
 */
bool lcd_getlr(uint8_t *line, uint8_t *row);        //-     Get line and row (target byte for line, target byte for row)

/**
 \brief Check if busy
 \retval true if busy
 \retval false if not busy
 */
bool lcd_busy(void);            //-     Check if busy

/**
 \brief Wait some microseconds
 \param us microsecond
 \return none
 */
void lcd_wait_us(unsigned short us);    //-     Wait some microseconds

/**
 \brief Wait some milliseconds
 \param ms milliseconds
 \return none
 */
void lcd_wait_ms(unsigned short ms);    //-     Wait some milliseconds


void lcd_backlight(int bl);     //

//-------------------------------------------------------------------------------------------------------------------
/*@}*/

#endif
