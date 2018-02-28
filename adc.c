/* ADC */

//#include <stdint.h>
#include <avr/io.h>


void adc_init() {
    ADMUX |= (1 << REFS0);
    ADCSRA = (1 << ADEN);       /* Enable ADC */
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);       /* Set base freq prescale */
}

uint16_t adc_read(uint8_t ch) {
    if (ch > 7)
        return 0;

    ch &= (1 << MUX2) | (1 << MUX1) | (1 << MUX0); //0b00000111;
    ADMUX = (ADMUX & 0xf8) | ch;        /* Channel selection */
    ADCSRA |= (1 << ADSC);              /* Start conversion */

    while (ADCSRA & (1 << ADSC));

    return (ADC);
}

/* EOF */
