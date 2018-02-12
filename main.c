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


#define BAUD 38400
#include <util/setbaud.h>

#include <fifo.h>
#include <tools.h>
#include <shell.h>


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
    wdt_enable(WDTO_30MS);
    WDTCSR = (1 << WDIE);
}

#define CLKD1    (1<<CS20)                      /* CLK/8 */
#define CLKD8    (1<<CS21)                      /* CLK/8 */
#define CLKD32   (1<<CS20) | (1<<CS21)          /* CLK/32 */
#define CLKD64   (1<<CS22)                      /* CLK/64 */
#define CLKD128  (1<<CS20) | (1<<CS22)          /* CLK/128 */
#define CLKD256  (1<<CS21) | (1<<CS22)          /* CLK/256 */
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

    OCR0A = 10;                 /* #6 */
    OCR0B = 10;                 /* #5 */
}

void pwm1_init(void) {

    /* Timer 1 */
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10) | (1 << WGM11);
    TCCR1B = (1 << WGM12) | (1 << CS10) | (1 << CS12);
    DDRB |= (1 << PORTB1) | (1 << PORTB2);

    OCR1A = 15;
    OCR1B = 15;
}


void pwm2_init(void) {
    /* Timer 2 */
    TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
    TCCR2B = (1 << CS20) | (1 << CS21) | (1 << CS22);
    DDRB |= (1 << PORTB3);
    DDRD |= (1 << PORTD3);

    OCR2A = 10;
    OCR2B = 10;
}

void adc_init() {
    ADMUX |= (1 << REFS0);
    ADCSRA = (1 << ADEN);                                  /* Enable ADC */
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  /* Set base freq prescale */
}

uint16_t adc_read(uint8_t ch) {

    if (ch > 7)
        return 0;

    ADMUX = (ADMUX & 0xF0) | ch;        /* Channel selection */
    ADCSRA |= (1 << ADSC);              /* Start conversion */

    while (!ADCSRA & (1 << ADIF));
    ADCSRA |= (1 << ADIF);

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
    outl("[s3|s5|s6] pos - set servo position");
    outl("help - this help");
    return 1;
}


int16_t cmd_hello(void) {
    outl("Hello!");
    return 1;
}



int16_t cmd_serv3(uint8_t * arg) {
    int8_t i = str2int(arg);
    if (i >= 10 && i <= 37) {
        OCR2B = i;
        return i;
    } else {
        return -1;
    }
}

int16_t cmd_serv5(uint8_t * arg) {
    int8_t i = str2int(arg);
    if (i >= 10 && i <= 37) {
        OCR0B = i;
        return i;
    } else {
        return -1;
    }
}

int16_t cmd_serv6(uint8_t * arg) {
    int8_t i = str2int(arg);
    if (i >= 10 && i <= 37) {
        OCR2A = i;
        return i;
    } else {
        return -1;
    }
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

cdef_t cdef[] = {
    {"help", &cmd_help, 0}
    ,
    {"hello", &cmd_hello, 0}
    ,
    {"s3", &cmd_serv3, 1}
    ,
    {"s5", &cmd_serv5, 1}
    ,
    {"s6", &cmd_serv6, 1}
    ,
    {"delay", &cmd_delay, 1}
    ,
    {"sleep", &cmd_delay, 1}
};

#define STR_LEN 64

uint8_t *prompt = "READY>";

int main() {

    fifo_iohook();
    uart_init();
    wdt_init();
    timer_init();
    pwm0_init();
    pwm1_init();
    pwm2_init();
    adc_init();
    sei();

    outl("\r\nTINY SHELL V01");
    outs(prompt);

    uint8_t str[STR_LEN];
    memset(str, 0, STR_LEN);

    _delay_ms(1000);

    while (1) {

        uint8_t *s;
        s = str;
        while (fifo_gett(in, str, STR_LEN, '\r') > 0) {

            shell(str, cdef, sizeof(cdef) / sizeof(cdef[0]));
            outs("\r\n");
            outs(prompt);

        }
        _delay_ms(100);
    }
}
