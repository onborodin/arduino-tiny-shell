/*
 $Id$
 */

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

#include <i2clcd.h>

static uint8_t backlight = OFF;

/*
    Example:
    screen_t screen;
    lcd_clear(&screen);
    lcd_home(&screen);
    lcd_printlr(&screen, 1, 3, "IOXX");
    lcd_printlr(&screen, 0, 5, "1234");
 */


/* Display initialization sequence */
void lcd_init(void) {
    {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)

    lcd_wait_ms(16);            /* Wait for more than 15ms after VDD rises to 4.5V */
    lcd_hw_write(LCD_D5 | LCD_D4); /* Set interface to 8-bit */
    lcd_wait_ms(5);             /* Wait for more than 4.1ms */
    lcd_hw_write(LCD_D5 | LCD_D4); /* Set interface to 8-bit */
    lcd_wait_us(101);           /* Wait for more than 100us      */
    lcd_hw_write(LCD_D5 | LCD_D4); /* Set interface to 8-bit */
    lcd_hw_write(LCD_D5);          /* Set interface to 4-bit */
    /* From now on in 4-bit-Mode */
    lcd_hw_command(LCD_4BIT | LCD_2LINE | LCD_5X7);        /* 2-Lines, 5x7-Matrix */
    lcd_hw_command(LCD_DISPLAYOFF);                        /* Display off */
    lcd_hw_command(LCD_CLEAR);                             /* Clear Screen */
    lcd_hw_command(LCD_INCREASE | LCD_DISPLAYSHIFTOFF);    /* Entrymode (Display Shift: off, Increment Address Counter) */
    lcd_hw_command(LCD_DISPLAYON);                         /* Display on */
    }
}

/* Write data to i2c */
void lcd_hw_write_i2c(uint8_t value) {
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
void lcd_hw_write(uint8_t value) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        lcd_hw_write_i2c(value | LCD_E);   /* Set enable to high */
        lcd_wait_us(252);
        lcd_hw_write_i2c(value | LCD_E);   /* Send data, keep enable high */
        lcd_wait_us(252);
        lcd_hw_write_i2c(value & ~LCD_E);  /* Set enable to low */
        lcd_wait_us(252);
    }
}

/* Issue a command to the display (use the defined commands above) */
void lcd_hw_command(uint8_t command) {
    uint8_t data;

    data = command & 0xF0;
    lcd_hw_write(data);

    data = (command << 4) & 0xF0;
    lcd_hw_write(data);
}

/* Go to position (line, row) */
bool lcd_hw_gotolr(uint8_t line, uint8_t row) {

    if (line > LCD_LINES)
        return false;
    if (row > LCD_ROWS)
        return false;

    uint8_t addr;
    switch(line) {
        case 0: 
            addr = LCD_LINE0 + row;
            break;
#if LCD_LINES>=2
        case 1: 
            addr = LCD_LINE1 + row;
            break;
#endif
#if LCD_LINES>=3
        case 2: 
            addr = LCD_LINE2 + row;
            break;
#endif
#if LCD_LINES>=4
        case 3: 
            addr = LCD_LINE4 + row;
            break;
#endif
        default: 
            addr = LCD_LINE0 + row;
    }
    lcd_hw_command( (1 << LCD_DDRAM) | addr);
    return true;
}


/* Put char to cursor position */
void lcd_hw_putchar(uint8_t value) {
    uint8_t data;

    data = value & 0xF0;
    data |= LCD_RS;
    lcd_hw_write(data);

    data = (value << 4) & 0xF0;
    data |= LCD_RS;
    lcd_hw_write(data);
}

/* Put char to position */
bool lcd_hw_putcharlr(uint8_t line, uint8_t row, uint8_t value) {
    if (!lcd_hw_gotolr(line, row))
        return false;
    lcd_hw_putchar(value);

    return true;
}

/* Clear screen buffer */
void lcd_clear(screen_t *screen) {
    uint8_t row, line;

    for(line = 0; line < LCD_LINES; line++) {
        for (row = 0; row < LCD_ROWS; row++) {
            screen->buf[line][row] = ' ';
        }
    }
    //lcd_render(screen);
}

/* Set cursor to home position */
void lcd_home(screen_t *screen) {
    uint8_t row, line;
    screen->line = 0;
    screen->row = 0;
}

/* Render screen buffer */
void lcd_render(screen_t *screen) {
    uint8_t row, line;

    for(line = 0; line < LCD_LINES; line++) {
        for (row = 0; row < LCD_ROWS; row++) {
            lcd_hw_putcharlr(line, row, screen->buf[line][row]);
        }
    }
}


/* Set cursor to position */
void lcd_pos(screen_t *screen, uint8_t line, uint8_t row) {

    screen->line = line;
    screen->row = row;

    if (line >= LCD_LINES)
        screen->line = LCD_LINES - 1;
    if (row >= LCD_ROWS)
        screen->row = LCD_ROWS - 1;
}

/* Print string to screen position */
bool lcd_printlr(screen_t *screen, uint8_t line, uint8_t row, uint8_t *string) {
    if (line >= LCD_LINES || row >= LCD_ROWS)
        return false;

    uint8_t i = 0;
    while (string[i] != 0 && (row + i) < LCD_ROWS) {
        screen->buf[line][row] = string[i];
        i++;
        row++;
    }
    //lcd_render(screen);
    return true;
}

/* Print char to screen position */
bool lcd_putclr(screen_t *screen, uint8_t line, uint8_t row, uint8_t data) {
    if (line >= LCD_LINES || row >= LCD_ROWS)
        return false;
    screen->buf[line][row] = data;
    lcd_render(screen);
    return true;
}

/* Print string with scrolling */
bool lcd_print(screen_t *screen, uint8_t *string) {
    uint8_t i;

    for(i = 0; string[i] != 0; i++) {
        screen->buf[screen->line][screen->row] = string[i];
        screen->row++;

        if (screen->row >= LCD_ROWS) {

            if (screen->line < (LCD_LINES - 1)) {
                /* If line is not last set cursor to next line */
                screen->line += 1;
                screen->row = 0;
            } else {

                uint8_t row, line;
                /* Line from 1 to last copy to up */
                for(line = 1; line < LCD_LINES; line++) {
                    for(row = 0; row < LCD_ROWS; row++) {
                        screen->buf[line - 1][row] = screen->buf[line][row];
                    }
                }
                /* Clean last line */
                for(row = 0; row < LCD_ROWS; row++) {
                    screen->buf[LCD_LINES - 1][row] = ' ';
                }
                screen->line = LCD_LINES - 1;
                screen->row = 0;
            }
        }
    }
    lcd_render(screen);
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

void lcd_hw_backlight(uint8_t bl) {
    backlight = bl;
}

/* Clear screen */
void lcd_hw_clear(void) {
    lcd_hw_command(LCD_CLEAR);
}
/* EOF */
