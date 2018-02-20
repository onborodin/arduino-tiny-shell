#include <avr/io.h>
#include <compat/twi.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/*
    Usage example, on master side.

    Write to slave device:

    #define SL_ADDR 0x43
    uint8_t regaddr = 0x01;
    uint8_t regdata = 0x78;
    i2c_start_wait((SL_ADDR << 1) | I2C_WRITE);
    data = i2c_write(regaddr);
    data = i2c_write(regdata);
    i2c_stop();

    Read from slave:

    #define SL_ADDR 0x43
    uint8_t regdaddr = 0x01;
    uint8_t regdata = 0;
    i2c_start_wait((SL_ADDR << 1) | I2C_WRITE));
    i2c_write(0x01);
    i2c_rep_start((SL_ADDR << 1) | I2C_READ);
    regdata = i2c_readNak();
    i2c_stop();

 */

#define I2CSLAVE_ADDR 0x43

#define PORT_DDR 0xB0         /* PORTB Settings */
#define PORT_IN  0xB1         /* Get PINB */
#define PORT_OUT 0xB2         /* Set PORTB */

#define REG_ARR_SIZE 16

uint8_t regaddr = 0;         /* Store the Requested Register Address */
uint8_t regdata[REG_ARR_SIZE];         /* Store the Register Address Data */
volatile uint8_t state;

#define ST_NONE_RCVD    0
#define ST_ADDR_RCVD    1
#define ST_DATA_RCVD    2

int8_t regdata_get(int8_t *regdata, uint8_t regdata_size, uint8_t addr) {
    if (addr < regdata_size) {
        return regdata[addr];  /* Direct mapping received address to array index */
    }
}

void regdata_put(int8_t *regdata, uint8_t regdata_size, uint8_t addr, int8_t data) {
    if (addr < regdata_size) {
        regdata[addr] = ++data; /* Some operation with received data, for example increment */
    }
}


ISR(TWI_vect) {
    uint8_t twi_status;

    /* Disable Global Interrupt */
    cli();

    /* Get TWI Status Register, mask the prescaler bits (TWPS1,TWPS0) */
    twi_status = TWSR & 0xF8;

    int8_t data;

    switch (twi_status) {
        case TW_SR_SLA_ACK:         /* 0x60: SLA+W received, ACK returned */
            break;

        case TW_SR_DATA_ACK:        /* 0x80: data received, ACK returned */
            switch (state) {
                case ST_NONE_RCVD:
                    regaddr = TWDR;
                    state = ST_ADDR_RCVD;
                    break;
                case ST_ADDR_RCVD:
                    data = TWDR;
                    regdata_put(regdata, sizeof(regdata), regaddr, data);
                    state = ST_NONE_RCVD;
                    break;
            }
            break;

        case TW_ST_SLA_ACK:        /* 0xA8: SLA+R received, ACK returned */
            if (state == ST_ADDR_RCVD) {
                TWDR = regdata_get(regdata, sizeof(regdata), regaddr);
                state = ST_NONE_RCVD;
            }
            break;


        case TW_BUS_ERROR:         /* 0x00: illegal start or stop condition */
            state = ST_NONE_RCVD;
            break;

        case TW_SR_STOP:           /* 0xA0: stop or repeated start condition received while selected */
        case TW_ST_DATA_ACK:       /* 0xB8: data transmitted, ACK received */
        case TW_ST_DATA_NACK:      /* 0xC0: data transmitted, NACK received */
        case TW_ST_LAST_DATA:      /* 0xC8: last data byte transmitted, ACK received */
        default:
            break;
    }
    /* Clear TWINT Flag */
    TWCR |= (1 << TWINT);
    /* Enable global Interrupt */
    sei();
}


void i2cs_init (void) {
    /* TWI Pull UP */
    PORTC |= ((1 << PINC4) | (1 << PINC5));

    /* Initial I2C Slave */
    TWAR = (I2CSLAVE_ADDR << 1) & 0xFE;         /* Set I2C Address, Ignore I2C General Address 0x00 */
    TWDR = 0x00;                                /* Default Initial Value */

    /* Start Slave Listening: Clear TWINT Flag, Enable ACK, Enable TWI, TWI Interrupt Enable */
    TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
}


int main(void) {

    i2cs_init();
    sei();

    while (1) {
        _delay_ms(10);
    }
}
/* EOF */
