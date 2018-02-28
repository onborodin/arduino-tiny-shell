/* SPI */
void spi_init(void) {
    DDRB &= ~( 1 << PB4);  /* Set PB4 as input */
    DDRB |= ((1 << PB2) | (1 << PB3) | (1 << PB5)); /* Set PB2/3/4 as output */

    SPCR = (1 << MSTR);  /* Set master mode */
    SPCR |= (1 << SPR1) | (1 << SPR0); /* Speed */

    SPSR |= (1 << SPI2X); /* Set duouble speed */

    SPCR |= (1 << SPIE); /* Enable interrupt */
    SPCR |= (1 << SPE); /* Enable SPI */
}

void spi_write(uint8_t data) {
    SPDR = data;
    while(!(SPSR & (1 << SPIF)));
}

uint8_t spi_read(void) {
    while(!(SPSR & (1 << SPIF)));
    return SPDR;
}
