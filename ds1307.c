/* $Id$ */

#define DS1307_ADDR      0x68
#define I2C_READY    0
#define I2C_FAIL     1

#include <twim.h>
#include <ds1307.h>

#define DS_SEC  0
#define DS_MIN  1
#define DS_HOUR 2


uint8_t ds_write(uint8_t reg, uint8_t data) {
    if(i2c_start((DS1307_ADDR << 1) | I2C_WRITE) == I2C_FAIL) 
        return(0);

    i2c_write(reg);
    i2c_write(data);
    i2c_stop();
    return(1);
}

uint8_t ds_read(uint8_t reg) {
    volatile uint8_t data;
    if(i2c_start((DS1307_ADDR << 1) | I2C_WRITE) == I2C_FAIL) 
        return(0);
    i2c_write(reg);
    i2c_rep_start((DS1307_ADDR << 1) | I2C_READ);
    data = i2c_readNak();
    i2c_stop();
    return data;
}

volatile uint8_t ds_bcd2dec(uint8_t val) {
    return(val - 6 * (val >> 4));
}

uint8_t ds_dec2bcd(uint8_t val) {
    return(val + 6 * (val / 10));
}

uint8_t ds_get_sec(void) {
    uint8_t r = ds_read(DS_SEC);
    r &= 0x7F;
    return(ds_bcd2dec(r));
}

uint8_t ds_get_min(void) {
    uint8_t r = ds_read(DS_MIN);
    return(ds_bcd2dec(r));
}

uint8_t ds_get_hour(void) {
    uint8_t r = ds_read(DS_HOUR);
    return(ds_bcd2dec(r));
}


void ds_set_sec(uint8_t num) {
    uint8_t r = ds_write(DS_SEC, ds_dec2bcd(num));
}

void ds_set_min(uint8_t num) {
    uint8_t r = ds_write(DS_MIN, ds_dec2bcd(num));
}

void ds_set_hour(uint8_t num) {
    uint8_t r = ds_write(DS_HOUR, ds_dec2bcd(num));
}

#define DS_OSC  7
#define DS_24H  6

void ds_init(void) {
    /* Start oscillator */
    uint8_t r;
    r  = ds_read(DS_SEC);
    r &= ~(1 << DS_OSC);
    ds_write(DS_SEC, r);

    /* Set 24h format */
    r = ds_read(DS_HOUR);
    r &= ~(1 << DS_24H);
    ds_write(DS_HOUR, r);
}


/* EOF */
