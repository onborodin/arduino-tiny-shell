/* $Id$ */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <tools.h>
#include <shell.h>
#include <fifo.h>



bool gettseq(uint8_t ** headp, uint8_t ** tailp, uint8_t c, uint8_t t) {
    if ((*headp)[0] == 0)
        return false;

    while ((*headp)[0] == c || (*headp)[0] == t)
        (*headp)++;
    if ((*headp)[0] == 0)
        return false;

    (*tailp) = (*headp);
    while ((*tailp)[0] != t && (*tailp)[0] != 0) {
        (*tailp)++;
    }
    if ((*tailp)[0] != t)
        return false;

    (*tailp)[0] = 0;
    (*tailp)++;
    return true;
}


bool getcseq(uint8_t ** headp, uint8_t ** tailp, uint8_t ** endp, const uint8_t c) {
    while ((*headp)[0] == c && (*headp)[0] != 0)
        (*headp)++;
    if ((*headp)[0] == 0)
        return false;

    (*tailp) = (*headp);
    while ((*tailp)[0] != c && (*tailp)[0] != 0) {
        (*tailp)++;
    }

    if ((*tailp) >= (*endp))
        return false;

    (*tailp)[0] = 0;
    (*tailp)++;
    return true;
}

void shell(uint8_t * str, cdef_t * cdef, uint8_t ccount) {

    uint8_t space = ' ', semic = ';', null = 0, newl = '\n';

    uint8_t *head = str;
    uint8_t *tail;

    while (gettseq(&head, &tail, space, semic)) {
        uint8_t *cmdhead = head;
        uint8_t *cmdtail;

        cmd_t cmd = { };
        cmd.argc = 0;

        while (getcseq(&cmdhead, &cmdtail, &tail, space) && (cmd.argc < MAX_ARGC)) {
            cmd.arg[cmd.argc] = cmdhead;
            cmd.argc++;
            cmdhead = cmdtail;
        };

        uint8_t i = 0, n = ccount, m = 0;

        int16_t r = 0;
        bool s = false;
        while (i < n) {
            if (str_cmp(cdef[i].name, cmd.arg[0])) {
                s = true;
                if (cmd.argc < cdef[i].argc) {
                    outl("ERR NOT ENOUG ARGS");
                    break;
                }
                switch (cdef[i].argc) {
                case 0:
                    r = (cdef[i].func) ();
                    break;
                case 1:
                    r = (cdef[i].func) (cmd.arg[1]);
                    break;
                case 2:
                    r = (cdef[i].func) (cmd.arg[1], cmd.arg[2]);
                    break;
                };
                break;
            }
            i++;
        }
        if (!s) {
            outl("ERR COMMAND NOT FOUND");
        } else if (r < 0) {
            printf("ERR %d", r);
            outl("");
        } else {
            printf("OK %d", r);
            outl("");
        }
        head = tail;
    }
}

//EOF
