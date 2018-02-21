/* $Id$ */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <tools.h>
#include <shell.h>
#include <fifo.h>

bool shell_get_cmd(uint8_t ** headp, uint8_t ** tailp, uint8_t c, uint8_t t) {
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


bool shell_get_args(uint8_t ** headp, uint8_t ** tailp, uint8_t ** endp, const uint8_t c) {
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

int8_t shell(uint8_t * str, act_t * act, uint8_t act_count) {

    uint8_t semic = ';';
    uint8_t space = ' ';

    uint8_t *head = str;
    uint8_t *tail;

    while (shell_get_cmd(&head, &tail, space, semic)) {
        uint8_t *cmd_head = head;
        uint8_t *cmd_tail;

        cmd_t cmd = { };
        cmd.argc = 0;

        while (shell_get_args(&cmd_head, &cmd_tail, &tail, space) && (cmd.argc < MAX_ARGC)) {
            cmd.arg[cmd.argc] = cmd_head;
            cmd.argc++;
            cmd_head = cmd_tail;
        };

        uint8_t i = 0;

        int16_t ret_code = 0;
        bool act_found = false;

        while (i < act_count) {
            if (str_cmp(act[i].name, cmd.arg[0])) {
                act_found = true;
                if (cmd.argc < act[i].argc) {
                    return SH_CMD_ARGCNT;
                    break;
                }
                switch (act[i].argc) {
                case 0:
                    ret_code = (act[i].func)();
                    break;
                case 1:
                    ret_code = (act[i].func)(cmd.arg[1]);
                    break;
                case 2:
                    ret_code = (act[i].func)(cmd.arg[1], cmd.arg[2]);
                    break;
                };
                break;
            }
            i++;
        }

        if (!act_found) {
            return SH_CMD_NOTFND;
        } else if (ret_code < 0) {
            return SH_CMD_ERROR;
        } else {
            return ret_code;
        }
        head = tail;
    }
    return SH_CMD_SUCCESS;
}

//EOF
