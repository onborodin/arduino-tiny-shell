

/* SPI */
void spi_init(void) {

    DDRB &= ~(1 << PB4); /* Set PB4 as input */
    DDRB |= ((1 << PB2) | (1 << PB3) | (1 << PB5)); /* Set PB2/3/4 as output */


    SPCR = (1 << MSTR); /* Set master mode */
    SPCR |= (1 << SPR1) | (1 << SPR0); /* Speed */
    //SPSR |= (1 << SPI2X); /* Set duouble speed */

    SPCR &= ~(1 << DORD); /* Set order */
    SPCR &= ~(1 << CPOL) | ~(1 << CPHA); /* Set mode */

    SPCR |= (1 << SPIE); /* Enable interrupt */
    SPCR |= (1 << SPE); /* Enable SPI */

}

uint8_t spi_read(uint8_t data) {
    uint8_t res;
    PORTB &= ~(1 << SPI_SS);
    SPDR = data;

    while (!(SPSR & (1 << SPIF)));
    res = SPDR;
    PORTB |= (1 << SPI_SS);

    return res;
}

void spi_write(uint8_t data) {
    PORTB &= ~(1 << SPI_SS);
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    PORTB |= (1 << SPI_SS);
}


void spi_write_array(uint8_t * data, uint8_t num) {
    PORTB &= ~(1 << SPI_SS);
    while (num--) {
        SPDR = *data++;
        while (!(SPSR & (1 << SPIF)));
    }
    PORTB |= (1 << SPI_SS);
}
