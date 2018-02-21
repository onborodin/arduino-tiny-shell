
/* $Id$ */

#ifndef SHELL_H_ITR
#define SHELL_H_ITR

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define MAX_ARGC 3
#define SH_CMD_NOTFND   -127
#define SH_CMD_ERROR    -126
#define SH_CMD_ARGCNT   -125
#define SH_CMD_SUCCESS  1

typedef struct cmd {
    uint8_t *arg[MAX_ARGC];
    uint8_t argc;
} cmd_t;

typedef int16_t(*funcp_t) ();

typedef struct act {
    uint8_t *name;
    funcp_t func;
    uint8_t argc;
} act_t;

int8_t shell(uint8_t * str, act_t * cmd, uint8_t act_count);
#endif

/* EOF */
