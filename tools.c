/* $Id$ */

#include <stdint.h>
#include <stdbool.h>
#include <tools.h>

uint16_t str_len(uint8_t * str) {
    uint16_t i = 0;
    while (str[i] != 0)
        i++;
    return i;
}

uint8_t* str_replc(uint8_t ** str, uint8_t c, uint8_t r) {
    uint16_t i = 0;
    while ((*str)[i] != 0) {
        if ((*str)[i] == c)
            (*str)[i] = r;
        i++;
    }
    return (*str);
}

bool str_cmp(uint8_t * str1, uint8_t * str2) {
    uint8_t i = 0;
    while (str1[i] != 0 && str2[i] != 0) {
        if ((str1[i] != str2[i]) || (str1[i + 1] != str2[i + 1]))
            return false;
        i++;
    }
    return true;
}

uint8_t *str_ltrim(uint8_t * str, uint8_t c) {
    while (str[0] == c && str[0] != 0)
        str++;
    return str;
}

uint8_t *str_trim(uint8_t * str, uint8_t c) {
    while (str[0] == c && str[0] != 0)
        str++;

    uint8_t i = str_len(str) - 1;
    while (str[i] == c && i > 0) {
        i--;
    }
    str[++i] = 0;
    return str;
}

int32_t int_pow(uint8_t n, uint8_t s) {
    int64_t i = 1;
    while (s--) {
        i = i * n;
    }
    return i;
}

int32_t str2int(uint8_t * str) {
    uint8_t l = str_len(str);
    uint8_t i = l;
    int16_t n = 0;
    while (i > 0) {
        if (str[i-1] <= '9' && str[i-1] >= '0')
            n += (str[i-1] - '0') * int_pow(10, l - i);
        i--;
    }
    if (str[0] == '-')
        n = -n;
    return n;
}

uint8_t int2str(int32_t num, uint8_t * str, uint8_t buf_len, int16_t base) {
    static char digits[] = "0123456789abcdefghijklmnopqrstuvwxyz";
    uint8_t i = 0, sign = 0;
    if (num < 0) {
        sign = '-';
        num = -num;
    }
    do {
        str[i++] = digits[num % base];
    } while ((num /= base) > 0 && i < buf_len);

    if (sign)
        str[i++] = '-';
    str[i] = 0;

    uint8_t len = i - 1;
    uint8_t c, b = 0;

    while (i-- && b < i) {
        c = str[b];
        str[b] = str[i];
        str[i] = c;
        b++;
    }
    return len;
}

uint8_t *tok_comp(uint8_t ** str, uint8_t c, uint8_t * end) {
    uint8_t *p = *str;
    if (p >= end)
        return 0;
    if (p[0] == 0)
        return 0;

    uint8_t i = 0;

    while (i < str_len(p)) {
        if (p[i] == c || p[i] == 0)
            break;
        i++;
    }
    p[i] = 0;

    (*str) += ++i;
    i = 0;

    while ((*str)[i] == c)
        (*str)++;
    return p;
}

/* EOF */
