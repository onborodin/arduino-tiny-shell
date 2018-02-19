/*

 */

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

#include <i2clcd.h>

static uint8_t backlight = OFF;

/* Display initialization sequence */
void lcd_init(void) {
    {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)

    lcd_wait_ms(16);            /* Wait for more than 15ms after VDD rises to 4.5V */
    lcd_write(LCD_D5 | LCD_D4); /* Set interface to 8-bit */
    lcd_wait_ms(5);             /* Wait for more than 4.1ms */
    lcd_write(LCD_D5 | LCD_D4); /* Set interface to 8-bit */
    lcd_wait_us(101);           /* Wait for more than 100us      */
    lcd_write(LCD_D5 | LCD_D4); /* Set interface to 8-bit */
    lcd_write(LCD_D5);          /* Set interface to 4-bit */
    /* From now on in 4-bit-Mode */
    lcd_command(LCD_4BIT | LCD_2LINE | LCD_5X7);        /* 2-Lines, 5x7-Matrix */
    lcd_command(LCD_DISPLAYOFF);                        /* Display off */
    lcd_command(LCD_CLEAR);                             /* Clear Screen */
    lcd_command(LCD_INCREASE | LCD_DISPLAYSHIFTOFF);    /* Entrymode (Display Shift: off, Increment Address Counter) */
    lcd_command(LCD_DISPLAYON);                         /* Display on */
    }
}

/* Write data to i2c */
void lcd_write_i2c(uint8_t value) {
    i2c_start_wait((LCD_I2C_DEVICE << 1) | I2C_WRITE);

    if (backlight) {
        value |= (1 << LCD_BL_PIN);
    } else {
        value &= ~(1 << LCD_BL_PIN);
    }
    i2c_write(value);
    i2c_stop();
}

/* Write byte to display with toggle of enable-bit */
void lcd_write(uint8_t value) {
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        lcd_write_i2c(value | LCD_E);   /* Set enable to high */
        lcd_wait_us(252);
        lcd_write_i2c(value | LCD_E);   /* Send data, keep enable high */
        lcd_wait_us(252);
        lcd_write_i2c(value & ~LCD_E);  /* Set enable to low */
        lcd_wait_us(252);
    }
}

/* Issue a command to the display (use the defined commands above) */
void lcd_command(uint8_t command) {
    uint8_t data;

    data = command & 0xF0;
    lcd_write(data);

    data = (command << 4) & 0xF0;
    lcd_write(data);
}

/* Put char to cursor position */
void lcd_putchar(uint8_t value) {
    uint8_t data;

    data = value & 0xF0;
    data |= LCD_RS;
    lcd_write(data);

    data = (value << 4) & 0xF0;
    data |= LCD_RS;
    lcd_write(data);
}

/* Print string to cursor position */
void lcd_print(uint8_t *string) {
    uint8_t i = 0;
    while (string[i] != 0x00) {
        lcd_putchar(string[i]);
        i++;
    }
}

/* Put char to position */
bool lcd_putcharlr(uint8_t line, uint8_t row, uint8_t value) {
    if (!lcd_gotolr(line, row))
        return false;
    lcd_putchar(value);

    return true;
}


/* Print string to position (If string is longer than LCD_ROWS overwrite first chars)(line, row, string) */
bool lcd_printlc(uint8_t line, uint8_t row, uint8_t *string) {
    uint8_t i;

    for (i = 0; string[i] != 0x00; i++) {
        if (!lcd_putcharlr(line, row, string[i]))
            return false;
        row++;
        if (row >= LCD_ROWS) {
            row = 0;
        }
    }
    return true;
}

/* Print string to position (If string is longer than LCD_ROWS continue in next line)(line, row, string) */
bool lcd_printlrc(uint8_t line, uint8_t row, uint8_t *string) {
    uint8_t i;

    for (i = 0; string[i] != 0x00; i++) {
        if (!lcd_putcharlr(line, row, string[i]))
            return false;
        row++;
        if (row >= LCD_ROWS) {
            line++;
            row = 0;
        }
        if (line >= LCD_LINES) {
            line = 0;
        }
    }
    return true;
}

/* Print string to position (line, row, string) */
bool lcd_printlr(uint8_t line, uint8_t row, uint8_t *string) {
    if (!lcd_gotolr(line, row))
        return false;
    lcd_print(string);

    return true;
}

/* Go to position (line, row) */
bool lcd_gotolr(uint8_t line, uint8_t row) {

    if (line > LCD_LINES)
        return false;
    if (row > LCD_ROWS)
        return false;
    //if ((line == 0) || (row == 0))
    //    return false;

    uint8_t addr;
    switch(line) {
        case 0: 
            addr = LCD_LINE1 + row;
            break;
#if LCD_LINES>=2
        case 1: 
            addr = LCD_LINE2 + row;
            break;
#endif
#if LCD_LINES>=3
        case 2: 
            addr = LCD_LINE2 + row;
            break;
#endif
#if LCD_LINES>=4
        case 3: 
            addr = LCD_LINE2 + row;
            break;
#endif
        default: 
            addr = LCD_LINE1 + row;
    }
    lcd_command( (1 << LCD_DDRAM) | addr);
    return true;
}

/* Wait some microseconds */
void lcd_wait_us(uint16_t us) {
    uint16_t i;
    for (i = 0; i < us; i++) {
        wait1us;
    }
}

/* Wait some milliseconds */
void lcd_wait_ms(uint16_t ms) {
    uint16_t i;
    for (i = 0; i < ms; i++) {
        wait1ms;
    }
}

void lcd_backlight(uint8_t bl) {
    backlight = bl;
}

/* Clear screen */
void lcd_clear(void) {
    lcd_command(LCD_CLEAR);
}

/* EOF */
