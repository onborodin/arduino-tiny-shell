
/* $Id$ */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <fifo.h>
#include <tools.h>

#ifndef FIFO_BUFFER_SIZE
#define FIFO_BUFFER_SIZE 128
#endif


void fifo_init(FIFO * b, uint8_t * buffer, uint8_t buffer_len) {
    if (b && buffer) {
        memset((void **)buffer, 0, buffer_len);
        b->buffer_len = buffer_len;
        b->buffer = buffer;
        b->head = 0;
        b->tail = 0;
    }
}

uint8_t fifo_count(const FIFO * b) {
    if (b) {
        return (b->head - b->tail);
    }
    return 0;
}

bool fifo_full(const FIFO * b) {
    if (b) {
        return (fifo_count(b) == b->buffer_len);
    }
    return true;
}

bool fifo_empty(const FIFO * b) {
    if (b) {
        return (fifo_count(b) == 0);
    }
    return true;
}

uint8_t fifo_peek(const FIFO * b) {
    uint8_t data = 0;

    if (!fifo_empty(b)) {
        data = b->buffer[b->tail % b->buffer_len];
    }
    return data;
}

bool fifo_back(FIFO * b) {
    if (!fifo_empty(b)) {
        b->head--;
        return true;
    }
    return false;
}

uint8_t fifo_getc(FIFO * b) {
    uint8_t data = 0;

    if (!fifo_empty(b)) {
        data = b->buffer[b->tail % b->buffer_len];
        b->tail++;
    }
    return data;
}

bool fifo_putc(FIFO * b, uint8_t data) {
    bool status = false;

    if (b) {
        if (!fifo_full(b)) {
            b->buffer[b->head % b->buffer_len] = data;
            b->head++;
            status = true;
        }
    }
    return status;
}

uint8_t fifo_puts(FIFO * b, uint8_t * string) {
    if (b) {
        for (uint8_t i = 0; i < str_len(string); i++) {
            if (!fifo_putc(b, string[i]))
                return i;
        }
    }
    return 0;
}

bool fifo_scanc(FIFO * b, uint8_t c) {
    if (b) {
        if (!fifo_empty(b)) {
            uint8_t tail = b->tail;

            for (uint8_t i = 0; i < fifo_count(b); i++) {
                uint8_t data = b->buffer[tail % b->buffer_len];

                if (data == c) {
                    return true;
                }
                tail++;
            }
        }
        return false;
    }
    return false;
}

uint8_t fifo_get_token(FIFO * b, uint8_t * str, uint8_t len, uint8_t term) {
    if (b) {
        memset((void *)str, 0, len);

        if (fifo_scanc(b, term) && str) {
            uint8_t i = 0, c = 0;

            while ((c = fifo_getc(b)) != 0 && c != term && i < len) {
                str[i] = c;
                i++;
            }
            return i;
        }
        return 0;
    }
    return 0;
}

/* EOF */
