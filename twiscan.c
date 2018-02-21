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

#define LCD_I2C_DEVICE  0x38
#include <i2clcd.h>

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
    UCSR0C &= ~(1 << USBS0) & ~(1 << UPM00) & ~(1 << UPM01);  /* One stop bit, no parity */

    UCSR0B |= (1 << TXEN0) | (1 << RXEN0);      /* Enable TX and RX */
    UCSR0B |= (1 << RXCIE0);    /* Enable Receive Interrupt */
    UCSR0B &= ~(1 << UDRIE0);   /* Disable Transmit Interrupt */
}

void wdt_init(void) {
    wdt_enable(WDTO_250MS);
    WDTCSR = (1 << WDIE);
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

#define DEC_STR_LEN 4
uint16_t twi_scan(void) {
    uint8_t addr = 1;
    uint8_t str[DEC_STR_LEN];
    fifo_puts(out, "\r\n");
    while (addr < 127) {
        if((addr == 0x38) || (addr == 0x7c))  {
            addr++;
            continue;
        }
        twi_start();
        twi_write(addr << 1);
        if(TW_STATUS == TW_MT_SLA_ACK ) {
            fifo_puts(out, "0x");
            int2str(addr, str, DEC_STR_LEN - 1, 16);
            fifo_puts(out, str);
        }
        twi_stop();
        addr++;
    }
    i2c_init();
    return addr;
}

act_t shell_act[] = {
    { .name = "ts", .func = &twi_scan, .argc = 0 }
};


#define MAX_CMD_LEN 164
uint8_t prompt[] = "READY>";

int main() {
    io_hook();
    uart_init();
    i2c_init();
    wdt_init();
    sei();

    _delay_ms(100);
    fifo_puts(out, prompt);

    uint8_t str[MAX_CMD_LEN];

    while (1) {
        while (fifo_get_token(in, str, MAX_CMD_LEN, '\r') > 0) {
            int8_t ret_code = shell(str, shell_act, sizeof(shell_act) / sizeof(shell_act[0]));
            if (ret_code == SH_CMD_NOTFND) 
                fifo_puts(out, (uint8_t *)"ERR COMMAND NOT FOUND\r\n");
            fifo_puts(out, prompt);
        }
        _delay_ms(100);
    }
}
/* EOF */
