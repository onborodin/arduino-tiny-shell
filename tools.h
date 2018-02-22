/* $Id$ */

#ifndef TOOLS_H_IRT
#define TOOLS_H_IRT

#define MAX_LINE_LEN 1024

uint16_t str_len(uint8_t * str);
bool str_cmp(uint8_t * str1, uint8_t * str2);
uint8_t *str_ltrim(uint8_t * str, uint8_t c);
uint8_t *str_trim(uint8_t * str, uint8_t c);
int32_t int_pow(uint8_t n, uint8_t s);
int32_t str2int(uint8_t * str);
uint8_t *int2str(int32_t num, uint8_t * str, uint8_t str_len, int16_t base);
uint8_t *str_endp(uint8_t * str);
uint8_t *tok_comp(uint8_t ** str, uint8_t c, uint8_t * end);

uint8_t * int2str_r(int32_t num, uint8_t * str, uint8_t buf_len, int16_t base);
uint8_t * str_toright(uint8_t *src, uint8_t *dst, uint8_t len_dst);

#endif
/* EOF */
