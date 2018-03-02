
/* $Id$ */

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/eeprom.h>

#define BAUD 19200
#include <util/setbaud.h>

#include <fifo.h>
#include <tools.h>
#include <shell.h>

#include <twim.h>

#define LCD_I2C_DEVICE  0x38
#include <i2clcd.h>

#include <md5.h>
#include <mpu6050.h>

#include <adc.h>

static uint8_t inbuf[FIFO_BUFFER_SIZE];
static uint8_t outbuf[FIFO_BUFFER_SIZE];

FIFO fifo_in, fifo_out;
FIFO *in, *out;

int uart_putchar(char c, FILE * stream) {
    return fifo_putc(&fifo_out, c);
}

int uart_getchar(FILE * stream) {
    return (int)fifo_getc(&fifo_out);
}

FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

void io_hook(void) {
    in = &fifo_in;
    out = &fifo_out;

    fifo_init(in, inbuf, sizeof(inbuf));
    fifo_init(out, outbuf, sizeof(outbuf));

    stdout = &uart_str;
    stdin = &uart_str;
    stderr = &uart_str;
}

/* UART */

void uart_init(void) {
    /* UBRR - USART Baud Rate Register */
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    /* UCSR  - USART Control and Status Register */
    /* U2X - Double Speed Operation */
    UCSR0A &= ~(1 << U2X0);

    /* UCSZ - USART Character Size, 8 bit */
    UCSR0B &= ~(1 << UCSZ02);
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);

    /* USBS - USART Stop Bit Select */
    /* UPM - USART Parity Mode */
    UCSR0C &= ~(1 << USBS0) & ~(1 << UPM00) & ~(1 << UPM01);    /* One stop bit, no parity */

    UCSR0B |= (1 << TXEN0) | (1 << RXEN0);      /* Enable TX and RX */
    UCSR0B |= (1 << RXCIE0);    /* Enable Receive Interrupt */
    UCSR0B &= ~(1 << UDRIE0);   /* Disable Transmit Interrupt */
}

ISR(USART_RX_vect) {
    volatile uint8_t ichar = UDR0;

    if (ichar == '\r') {
        fifo_putc(in, '\n');
        fifo_putc(out, '\n');
    }

    fifo_putc(in, ichar);
    fifo_putc(out, ichar);
}

/* Watchdog timer */
void wdt_init(void) {
    wdt_enable(WDTO_30MS);
    WDTCSR = (1 << WDIE);
}

ISR(WDT_vect) {
    wdt_reset();
    volatile uint8_t c;
    while ((c = fifo_getc(out)) > 0) {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = c;
    }
    WDTCSR = (1 << WDIE);
}

/* Shell */

act_t shell_act[] = {
};

/* Timer0 */
void timer0_init(void) {
    TCCR0B |= (1 << CS02) | (1 << CS00); /* CLK/1024 */
    //TCCR0B |= (1 << CS02);                 /* CLK/256 */
    TIMSK0 |= (1 << TOIE0);
}

/* Timer 0 */
ISR(TIMER0_OVF_vect) {
    #if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2
    mpu6050_update_quaternion();
    #endif
}

/* Timer1 */
void timer1_init(void) {
    TCCR1A = 0;
    TCCR1B = (1 << CS11);
    TIMSK1 = (1 << TOIE2);
}

void timer1_reset(void) {
    TCNT1 = 0;
}

ISR(TIMER1_OVF_vect) {
}


/* INT 0 */
void int0_init(void) {
    EICRA = (1 << ISC00);
    EIMSK |= (1 << INT0);
    DDRD &= ~(1 << PD2);
}

ISR(INT0_vect) {
}


#define MAX_CMD_LEN 164
#define MAX_LOGIN_LEN 8
#define MAX_PASSW_LEN 8
#define MAX_LOGIN_COUNT 4

typedef struct pwd {
    uint8_t login[MAX_LOGIN_LEN + 1];
    md5_hashstr_t passw;
} pwd_t;


int main() {
    io_hook();
    uart_init();
    i2c_init();
    wdt_init();
    lcd_init();
    adc_init();
    sei();
    _delay_ms(2000);

    uint8_t str[MAX_CMD_LEN];
    uint8_t prompt[] = "READY>";

    mpu6050_init();
    #if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2
    timer0_init();
    #endif
    //timer1_init();

    int0_init();

    screen_t screen;
    lcd_clear(&screen);
    lcd_home(&screen);

    //if (mpu6050_test_connection()) 
    //    lcd_print(&screen, prompt);

    fifo_puts(out, prompt);

    while (1) {

        #if MPU6050_GETATTITUDE == 0
        int16_t ax, ay, az, gx, gy, gz;
        uint8_t axs[5], ays[5], azs[5], gxs[5], gys[5], gzs[5];

        mpu6050_get_raw_data(&ax, &ay, &az, &gx, &gy, &gz);

        int2str_r(ax / 180, axs, 4, 10);
        int2str_r(ay / 180, ays, 4, 10);
        int2str_r(az / 180, azs, 4, 10);
        lcd_clear(&screen);
        lcd_printlr(&screen, 0, 0, axs);
        lcd_printlr(&screen, 0, 5, ays);
        lcd_printlr(&screen, 0, 10, azs);
        lcd_render(&screen);

        //printf("%6d %6d %6d - %6d %6d %6d\r\n", ax/182, ay/182, az/182, gx/10, gy/10, gz/10);
        #endif

        #if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2

        double roll, pitch, yaw;
        uint8_t roll_str[8], pitch_str[8], yaw_str[8];

        mpu6050_get_roll_pitch_yaw(&roll, &pitch, &yaw);

        snprintf(roll_str, 6, "%+5.0f", roll * (240/M_PI)/1.27);
        snprintf(pitch_str, 6, "%+5.0f", pitch * (240/M_PI)/1.27);
        snprintf(yaw_str, 6, "%+5.0f", yaw * (240/M_PI)/1.27);
        lcd_clear(&screen);
        lcd_printlr(&screen, 0, 0, pitch_str);
        lcd_printlr(&screen, 0, 7, roll_str);
        lcd_printlr(&screen, 1, 3, yaw_str);
        lcd_render(&screen);
        #endif

        while (fifo_get_token(in, str, MAX_CMD_LEN, '\r') > 0) {
            int8_t ret_code = shell(str, shell_act, sizeof(shell_act) / sizeof(shell_act[0]));
            if (ret_code == SH_CMD_NOTFND)
                fifo_puts(out, (uint8_t *) "COMMAND NOT FOUND\r\n");
            fifo_puts(out, prompt);
        }

        _delay_ms(100);
    }

}
/* EOF */
