/* $Id$ */

#ifndef DC1307_H_IUI
#define DC1307_H_IUI

uint8_t ds_write(uint8_t reg, uint8_t data);
uint8_t ds_read(uint8_t reg);

volatile uint8_t ds_bcd2dec(uint8_t val);
uint8_t ds_dec2bcd(uint8_t val);

uint8_t ds_get_sec(void);
uint8_t ds_get_min(void);
uint8_t ds_get_hour(void);

void ds_set_sec(uint8_t num);
void ds_set_min(uint8_t num);
void ds_set_hour(uint8_t num);

void ds_init(void);


#endif

/* EOF */
