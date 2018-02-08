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


void init_uart(void) {
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
    UCSR0C &= ~(1 << USBS0) & ~(1 << UPM00) & ~(1 << UPM01);
    UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
    UCSR0B |= (1 << RXCIE0);    /* Enable Receive Interrupt */
    UCSR0B &= ~(1 << UDRIE0);   /* Disabale Transmit Interrupt */
}

void wdt_init(void) {
    wdt_enable(WDTO_30MS);
    WDTCSR = (1 << WDIE);
}


ISR(USART_RX_vect) {
    uint8_t c = UDR0;
    if (c == '\r') {
        fifo_putc(in, '\n');
        fifo_putc(out, '\n');
    }
    fifo_putc(in, c);
    fifo_putc(out, c);

}

ISR(WDT_vect) {
    wdt_reset();
    uint8_t c;
    while ((c = fifo_getc(out)) > 0) {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = c;
    }
    WDTCSR = (1 << WDIE);
}

#define CLKD1    (1<<CS20)      /* CLK/8 */
#define CLKD8    (1<<CS21)      /* CLK/8 */
#define CLKD32   (1<<CS20)|(1<<CS21)    /* CLK/32 */
#define CLKD64   (1<<CS22);     /* CLK/64 */
#define CLKD128  (1<<CS20)|(1<<CS22)    /* CLK/128 */
#define CLKD256  (1<<CS21)|(1<<CS22)    /* CLK/256 */
#define CLKD1024 (1<<CS20)|(1<<CS21)|(1<<CS22)  /* CLK/1024 */

void timer_init(void) {
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR2B = CLKD256;
    TIMSK2 |= (1 << TOIE2);
}

void pwm_init(void) {
    TCCR0A |= _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
    TCCR0B = CLKD128;
    OCR0A = 30;
    OCR0B = 30;
    DDRB |= (1 << PORTB3);
    DDRD |= (1 << PORTD3) | (1 << PORTD6) | (1 << PORTD5);
}

ISR(TIMER2_OVF_vect) {
/* Dummy code */
}

int16_t cmd_hello(void) {
    outl("Hello!");
    return 1;
}

int16_t cmd_serv(uint8_t * arg) {
    int16_t i = str2int(arg);
//    int2str(i, s, STR_LEN, 10);

    outl(arg);

    if (i >= 10 && i <= 37) {
        OCR0A = i;
        OCR0B = i;
        return i;
    } else {
        outl("OUT OF RANGE 10-37");
        return -1;
    }
}

int16_t cmd_delay(uint8_t * arg) {
    int16_t i = str2int(arg);
    _delay_ms(1);
    return i;
}

cdef_t cdef[] = {
    {"hello", &cmd_hello, 0},
    {"serv", &cmd_serv, 1},
    {"delay", &cmd_delay, 1},
    {"sleep", &cmd_delay, 1}
};

#define STR_LEN 64

int main() {

    fifo_iohook();
    init_uart();
    wdt_init();
    timer_init();
    pwm_init();
    sei();

    _delay_ms(1000);

    outs("READY>");

    uint8_t str[STR_LEN];
    memset(str, 0, STR_LEN);

    while (1) {
        uint8_t *s;
        s = str;

        while (fifo_gett(in, str, STR_LEN, '\r') > 0) {

            shell(str, cdef, sizeof(cdef) / sizeof(cdef[0]));
            outs("\r\nREADY>");

        }
        _delay_ms(100);

    }
}
