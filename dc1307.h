/* $Id$ */

#ifndef DC1307_H_IUI
#define DC1307_H_IUI

uint8_t dc_write(uint8_t reg, uint8_t data);
uint8_t dc_read(uint8_t reg);

volatile uint8_t dc_bcd2dec(uint8_t val);
uint8_t dc_dec2bcd(uint8_t val);

uint8_t dc_get_sec(void);
uint8_t dc_get_min(void);
uint8_t dc_get_hour(void);

void dc_set_sec(uint8_t num);
void dc_set_min(uint8_t num);
void dc_set_hour(uint8_t num);

void dc_init(void);


#endif

/* EOF */
