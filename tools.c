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

uint8_t *str_replc(uint8_t ** str, uint8_t c, uint8_t r) {
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

uint8_t *str_copy(uint8_t * src, uint8_t * dst, uint8_t len) {
    uint8_t i = 0;
    while (src[i] != 0 && i < len) {
        dst[i] = src[i];
        i++;
    }
    dst[i] = 0;
    return dst;
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
        if (str[i - 1] <= '9' && str[i - 1] >= '0')
            n += (str[i - 1] - '0') * int_pow(10, l - i);
        i--;
    }
    if (str[0] == '-')
        n = -n;
    return n;
}

uint8_t *int2str(int32_t num, uint8_t * dst, uint8_t dst_len, int16_t base) {
    const uint8_t digits[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

    uint8_t i = 0, sign = 0;

    if (num < 0) {
        sign = '-';
        num = -num;
    }

    do {
        dst[i++] = digits[num % base];
    } while ((num /= base) > 0 && i < dst_len);

    if (sign > 0)
        dst[i++] = '-';
    dst[i] = 0;

    uint8_t c, b = 0;
    while (i-- && b < i) {
        c = dst[b];
        dst[b] = dst[i];
        dst[i] = c;
        b++;
    }
    return dst;
}

uint8_t *uint2str(uint32_t num, uint8_t * dst, uint8_t dst_len, int16_t base) {
    const uint8_t digits[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    uint8_t i = 0;

    do {
        dst[i++] = digits[num % base];
    } while ((num /= base) > 0 && i < dst_len);

    dst[i] = 0;

    uint8_t c, b = 0;
    while (i-- && b < i) {
        c = dst[b];
        dst[b] = dst[i];
        dst[i] = c;
        b++;
    }
    return dst;
}

uint8_t *int2str_r(int32_t num, uint8_t * dst, uint8_t dst_len, int16_t base) {
    const uint8_t digits[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    uint8_t i;
    for(i = 0; i < dst_len; i++) {
        dst[i] = ' ';
    }

    uint8_t sign = 0;
    dst[dst_len] = 0;

    if (num < 0) {
        sign = '-';
        num = -num;
    }

    i = dst_len;
    do {
        dst[--i] = digits[num % base];
    } while ((num /= base) > 0 && i > 0);

    if (sign > 0 && i > 0)
        dst[--i] = '-';

    return dst;
}

uint8_t *uint2str_r(int32_t num, uint8_t * dst, uint8_t dst_len, int16_t base) {
    const uint8_t digits[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    uint8_t i;
    for(i = 0; i < dst_len; i++) {
        dst[i] = ' ';
    }

    dst[dst_len] = 0;

    i = dst_len;
    do {
        dst[--i] = digits[num % base];
    } while ((num /= base) > 0 && i > 0);

    return dst;
}

uint8_t * str_toright(uint8_t *src, uint8_t *dst, uint8_t len_dst) {
    uint8_t len_src = str_len(src);
    uint8_t i, shift;
    for(i = 0; i < len_dst; i++)
        dst[i] = ' ';

    if (len_dst > len_src) {
        shift = len_dst - len_src;
        i = len_src + 1;
        while (i >= 0 && i <= len_src + 1) {
            dst[i + shift] = src[i];
            i--;
        }
    } else {
        i = len_dst + 1;
        shift = len_src - len_dst;
        while (i >= 0 && i <= len_dst + 1) {
            dst[i] = src[i + shift];
            i--;
        }
    }
    return dst;
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
