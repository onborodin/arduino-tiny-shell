
/* $Id$ */

#ifndef SHELL_H_ITR
#define SHELL_H_ITR

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define MAX_ARGC 4

typedef struct cmd {
    uint8_t *arg[MAX_ARGC];
    uint8_t argc;
} cmd_t;

typedef int16_t(*funcp_t) ();

typedef struct cdef {
    uint8_t *name;
    funcp_t func;
    uint8_t argc;
} cdef_t;

void shell(uint8_t * str, cdef_t * cdef, uint8_t ccount);
#endif

/* EOF */
