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
    wdt_enable(WDTO_30MS);
    WDTCSR = (1 << WDIE);
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

ISR(WDT_vect) {
    wdt_reset();
    volatile uint8_t c;
    while ((c = fifo_getc(out)) > 0) {
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = c;
    }
    WDTCSR = (1 << WDIE);
}

act_t shell_act[] = {
};

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

void int0_init(void) {
    EICRA = (1 << ISC00);
    EIMSK |= (1 << INT0);
    DDRD &= ~(1 << PD2);
}

void adc_init() {
    ADMUX |= (1 << REFS0);
    ADCSRA = (1 << ADEN);       /* Enable ADC */
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);       /* Set base freq prescale */
}

uint16_t adc_read(uint8_t ch) {
    if (ch > 7)
        return 0;

    ch &= (1 << MUX2) | (1 << MUX1) | (1 << MUX0); //0b00000111;
    ADMUX = (ADMUX & 0xf8) | ch;        /* Channel selection */
    ADCSRA |= (1 << ADSC);              /* Start conversion */

    while (ADCSRA & (1 << ADSC));

    return (ADC);
}



void spi_init(void) {
    DDRB &= ~( 1 << PB4);  /* Set PB4 as input */
    DDRB |= ((1 << PB2) | (1 << PB3) | (1 << PB5)); /* Set PB2/3/4 as output */

    SPCR = (1 << MSTR);  /* Set master mode */
    SPCR |= (1 << SPR1) | (1 << SPR0); /* Speed */

    SPSR |= (1 << SPI2X); /* Set duouble speed */

    SPCR |= (1 << SPIE); /* Enable interrupt */
    SPCR |= (1 << SPE); /* Enable SPI */

}

void spi_write(uint8_t data) {
    SPDR = data;
    while(!(SPSR & (1 << SPIF)));
}

uint8_t spi_read(void) {
    while(!(SPSR & (1 << SPIF)));
    return SPDR;
}

void eeprom_write(uint16_t address, uint8_t data) {
    /* Wait for completion of previous write */
    while(EECR & (1 << EEPE));
    /* Set up address and Data Registers */
    EEAR = address;
    EEDR = data;
    /* Write logical one to EEMPE */
    EECR |= (1 << EEMPE);
    /* Start eeprom write by setting EEPE */
    EECR |= (1 << EEPE);
}


uint8_t eeprom_read(uint16_t address) {
    /* Wait for completion of previous write */
    while(EECR & (1 << EEPE));
    /* Set up address register */
    EEAR = address;
    /* Start eeprom read by writing EERE */
    EECR |= (1 << EERE);
    /* Return data from Data Register */
    return EEDR;
}


#define MAX_CMD_LEN 164
#define MAX_LOGIN_LEN 8
#define MAX_PASSW_LEN 8
#define MAX_LOGIN_COUNT 4

/*
    uint8_t passw[] = "123";
    md5_hash_t hash;
    md5_hashstr_t hashstr;

    md5(&hash, passw, str_len(passw) * 8);
    md5_tohex(hash, hashstr);

    fifo_puts(out, hashstr);
 */

typedef uint8_t md5_hashstr_t[MD5_HASH_BYTES * 2 + 1];

void md5_tohex(md5_hash_t hash, md5_hashstr_t hashstr) {
    const uint8_t digits[] = "0123456789abcdef";
    uint8_t i;
    for(i = 0; i < sizeof(md5_hashstr_t); i++)
        hashstr[i] = 0;
    for(i = 0; i < MD5_HASH_BYTES; i++) {
            hashstr[i * 2] = digits[hash[i] >> 4];
            hashstr[i * 2 + 1] = digits[hash[i] & 0x0F];
    }
}


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
    //timer1_init();
    //int0_init();
    sei();
    _delay_ms(2000);

    uint8_t str[MAX_CMD_LEN];
    uint8_t prompt[] = "READY>";

    screen_t screen;
    lcd_clear(&screen);
    lcd_home(&screen);
    lcd_print(&screen, prompt);

    fifo_puts(out, prompt);

    while (1) {
        while (fifo_get_token(in, str, MAX_CMD_LEN, '\r') > 0) {
            int8_t ret_code = shell(str, shell_act, sizeof(shell_act) / sizeof(shell_act[0]));
            if (ret_code == SH_CMD_NOTFND) 
                fifo_puts(out, (uint8_t *)"COMMAND NOT FOUND\r\n");
            fifo_puts(out, prompt);
        }
        _delay_ms(100);
    }

}
