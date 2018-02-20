#include <util/twi.h>

#define SLA_W(address)          (address << 1)
#define SLA_R(address)  ((address << 1) + 0x01)

void twi_init(uint8_t twbr_value) {
    //TWSR = 0x00;
    TWBR = twbr_value;
    //pullups?
}

void twi_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void twi_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while ((TWCR & (1 << TWSTO)));
}

void twi_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t twi_read(uint8_t ack)
{
    TWCR = ack ? ((1 << TWINT) | (1 << TWEN) | (1 << TWEA))
        : ((1 << TWINT) | (1 << TWEN));

    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t twi_read_ACK(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);

    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

uint8_t twi_read_NACK(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

void twi_scan(void) {

    uint8_t a = 1;
    while (a < 127) {
        twi_start();
        twi_write(SLA_W(a));
        if(TW_STATUS == TW_MT_SLA_ACK ) {
            printf("0x%x\r\n", a);
        }
        twi_stop();
        a++;
    }
}

