/* $Id$ */

#ifndef UART_H_IUI
#define UART_H_IUI

#define FIFO_BUFFER_SIZE 128
//extern static uint8_t inbuf[FIFO_BUFFER_SIZE];
//extern static uint8_t outbuf[FIFO_BUFFER_SIZE];

typedef struct fifo {
    volatile unsigned head;
    volatile unsigned tail;
    volatile uint8_t *buffer;
    unsigned buffer_len;
} FIFO;

extern FIFO fifo_in, fifo_out;
extern FIFO *in, *out;

void fifo_iohook(void);

void fifo_init(FIFO * b, uint8_t * buffer, uint8_t buffer_len);
uint8_t fifo_count(const FIFO * b);
static bool fifo_full(const FIFO * b);
bool fifo_empty(const FIFO * b);
uint8_t fifo_peek(const FIFO * b);
uint8_t fifo_getc(FIFO * b);
bool fifo_putc(FIFO * b, uint8_t data);
uint8_t fifo_puts(FIFO * b, uint8_t * str);
bool fifo_scanc(FIFO * b, uint8_t c);
uint8_t fifo_gett(FIFO * b, uint8_t * str, uint8_t len, uint8_t);

void outc(uint8_t c);
void outs(uint8_t * str);
void outl(uint8_t * str);


#endif
/* EOF */
