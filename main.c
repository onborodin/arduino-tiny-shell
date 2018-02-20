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


#define BAUD 19200
#include <util/setbaud.h>

#include <fifo.h>
#include <tools.h>
#include <shell.h>

#include <twim.h>
#include <ds1307.h>

#define LCD_I2C_DEVICE  0x38
#include <i2clcd.h>

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
    UCSR0C &= ~(1 << USBS0) /* One Stop Bit */ &~(1 << UPM00) & ~(1 << UPM01);  /* No Parity */

    UCSR0B |= (1 << TXEN0) | (1 << RXEN0);      /* Enable TX and RX */
    UCSR0B |= (1 << RXCIE0);    /* Enable Receive Interrupt */
    UCSR0B &= ~(1 << UDRIE0);   /* Disable Transmit Interrupt */
}

void wdt_init(void) {
    wdt_enable(WDTO_250MS);
    WDTCSR = (1 << WDIE);
}

#define CLKD1    (1<<CS20)      /* CLK/8 */
#define CLKD8    (1<<CS21)      /* CLK/8 */
#define CLKD32   (1<<CS20) | (1<<CS21)  /* CLK/32 */
#define CLKD64   (1<<CS22)      /* CLK/64 */
#define CLKD128  (1<<CS20) | (1<<CS22)  /* CLK/128 */
#define CLKD256  (1<<CS21) | (1<<CS22)  /* CLK/256 */
#define CLKD1024 (1<<CS20) | (1<<CS21) | (1<<CS22)      /* CLK/1024 */

void timer_init(void) {
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR2B = CLKD256;
    TIMSK2 |= (1 << TOIE2);
}

void pwm0_init(void) {
    /* Timer 0 */
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00) | (1 << WGM01);
    TCCR0B = CLKD128;
    DDRD |= (1 << PORTD5) | (1 << PORTD6);

    OCR0A = 24;         /* #6 */
    OCR0B = 24;         /* #5 */
}

void pwm1_init(void) {

    /* Timer 1 */
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10) | (1 << WGM11);
    TCCR1B = (1 << WGM12) | (1 << CS10) | (1 << CS12);
    DDRB |= (1 << PORTB1) | (1 << PORTB2);

    OCR1A = 24;
    OCR1B = 24;
}


void pwm2_init(void) {
    /* Timer 2 */
    TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
    TCCR2B = (1 << CS20) | (1 << CS21) | (1 << CS22);
    DDRB |= (1 << PORTB3);
    DDRD |= (1 << PORTD3);

    OCR2A = 24;;
    OCR2B = 24;;
}

void adc_init() {
    ADMUX |= (1 << REFS0);
    ADCSRA = (1 << ADEN);       /* Enable ADC */
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);       /* Set base freq prescale */
}

#define CHANNEL_MODE 0x07

uint16_t adc_read(uint8_t ch) {
    if (ch > 6)
        return 0;

    ch &= CHANNEL_MODE;
    ADMUX = (ADMUX & 0xF8) | ch;        /* Channel selection */
    ADCSRA |= (1 << ADSC);              /* Start conversion */

    //while (!ADCSRA & (1 << ADIF));
    //ADCSRA |= (1 << ADIF);
    while (ADCSRA & (1 << ADSC));

    return (ADC);
}


ISR(TIMER2_OVF_vect) {
/* Dummy code */
}

ISR(USART_RX_vect) {
    volatile uint8_t c = UDR0;

    if (c == '\r') {
        fifo_putc(in, '\n');
        fifo_putc(out, '\n');
    }

    fifo_putc(in, c);
    fifo_putc(out, c);
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

int16_t cmd_help(void) {
    outl("Available commands:");
    outl("[s3|s5|s6|s9|s10|s11] pos - set servo position");
    outl("d n - delay n*0.1s");
    outl("h - this help");
    return 1;
}

int16_t cmd_serv3(uint8_t * arg) {
    int8_t i = str2int(arg);
    OCR2B = i;
    return i;
}

int16_t cmd_serv5(uint8_t * arg) {
    int8_t i = str2int(arg);
    OCR0B = i;
    return i;
}

int16_t cmd_serv6(uint8_t * arg) {
    int8_t i = str2int(arg);
    OCR0A = i;
    return i;
}

int16_t cmd_serv9(uint8_t * arg) {
    int8_t i = str2int(arg);
    OCR1A = i;
    return i;
}

int16_t cmd_serv10(uint8_t * arg) {
    int8_t i = str2int(arg);
    OCR1B = i;
    return i;
}

int16_t cmd_serv11(uint8_t * arg) {
    int8_t i = str2int(arg);
    OCR2A = i;
    return i;
}


int16_t cmd_delay(uint8_t * arg) {
    int16_t n = str2int(arg), i = n;
    if (i < 0)
        i = -i;
    while (i > 0) {
        _delay_ms(100);
        i--;
    }
    return n;
}


void twi_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void twi_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while ((TWCR & (1 << TWSTO)));
}

void twi_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint16_t twi_scan(void) {
    uint8_t addr = 1;
    uint8_t str[4];
    outnl();
    while (addr < 127) {
        if((addr == 0x38) || (addr == 0x7c))  {
            addr++;
            continue;
        }
        twi_start();
        twi_write(addr << 1);
        if(TW_STATUS == TW_MT_SLA_ACK ) {
            outs("0x");
            int2str(addr, str, 3, 16);
            outl(str);
        }
        twi_stop();
        addr++;
    }
    return addr;
}


uint16_t twi_scan_lcd(void) {
    uint8_t addr = 1;
    uint8_t str[4];
    lcd_gotolr(1,1);
    while (addr < 127) {
        if((addr == 0x38) || (addr == 0x7c))  {
            addr++;
            continue;
        }
        twi_start();
        twi_write(addr << 1);
        if(TW_STATUS == TW_MT_SLA_ACK ) {
            int2str(addr, str, 3, 16);
            lcd_print(str);
            lcd_putchar(' ');
        }
        twi_stop();
        addr++;
    }
    return addr;
}


cdef_t cdef[] = {
    {.name = "h",.func = &cmd_help,.argc = 0},
    {.name = "s3",.func = &cmd_serv3,.argc = 1}
    ,
    {.name = "s5",.func = &cmd_serv5,.argc = 1}
    ,
    {.name = "s6",.func = &cmd_serv6,.argc = 1}
    ,
    {.name = "s9",.func = &cmd_serv9,.argc = 1}
    ,
    {.name = "s10",.func = &cmd_serv10,.argc = 1}
    ,
    {.name = "s11",.func = &cmd_serv11,.argc = 1}
    ,
    {.name = "m1",.func = &cmd_serv3,.argc = 1}
    ,
    {.name = "m2",.func = &cmd_serv5,.argc = 1}
    ,
    {.name = "m3",.func = &cmd_serv6,.argc = 1}
    ,
    {.name = "m4",.func = &cmd_serv9,.argc = 1}
    ,
    {.name = "d",.func = &cmd_delay,.argc = 1},
    {.name = "ts",.func = &twi_scan,.argc = 0}
};


#define STR_LEN 164

uint8_t *prompt = "#READY>";


int main() {

    fifo_iohook();
    uart_init();
    timer_init();
    //pwm0_init();
    //pwm1_init();
    //pwm2_init();
    //adc_init();

    i2c_init();
    //ds_init();

    _delay_ms(100);
    lcd_init();
    wdt_init();

    sei();

    lcd_printlrc(0, 0, "########");

    _delay_ms(500);

    outl("\r\nTINY SHELL V01");
    outs(prompt);


    uint8_t str[STR_LEN];
    memset(str, 0, STR_LEN);

    _delay_ms(100);


#define SL_ADDR 0x43

    while (1) {

        uint8_t data = 0;
        lcd_clear();


        //uint8_t sec = ds_get_sec();
        //uint8_t strsec[12];
        //int2str(sec, strsec, 11, 16);
        //lcd_printlrc(0, 0, strsec);

        i2c_start_wait((SL_ADDR << 1) | I2C_WRITE);
            data = i2c_write(0x01);
            data = i2c_write(0x78);
            i2c_stop();



        if(!i2c_start((SL_ADDR << 1) | I2C_WRITE)) {
            i2c_write(0x01);
            i2c_rep_start((SL_ADDR << 1) | I2C_READ);
            data = i2c_readNak();
            i2c_stop();

            uint8_t str1[12];
            int2str(data, str1, 10, 16);
            lcd_printlrc(0, 0, str1);
        }

        uint8_t *s;
        s = str;

        while (fifo_gett(in, str, STR_LEN, '\r') > 0) {

            shell(str, cdef, sizeof(cdef) / sizeof(cdef[0]));
            outs("\r\n");
            outs(prompt);

        }
        _delay_ms(500);
    }

}
