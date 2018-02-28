/* $Id$ */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define CLKD1    (1<<CS20)              /* CLK/8 */
#define CLKD8    (1<<CS21)              /* CLK/8 */
#define CLKD32   (1<<CS20) | (1<<CS21)  /* CLK/32 */
#define CLKD64   (1<<CS22)              /* CLK/64 */
#define CLKD128  (1<<CS20) | (1<<CS22)  /* CLK/128 */
#define CLKD256  (1<<CS21) | (1<<CS22)  /* CLK/256 */
#define CLKD1024 (1<<CS20) | (1<<CS21) | (1<<CS22)      /* CLK/1024 */

void timer_init(void) {
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR2B = CLKD256;
    TIMSK2 |= (1 << TOIE2);
}

void pwm0_init(void) {
    /* Timer 0 */
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00) | (1 << WGM01);
    TCCR0B = CLKD128;
    DDRD |= (1 << PORTD5) | (1 << PORTD6);

    OCR0A = 24;                 /* #6 */
    OCR0B = 24;                 /* #5 */
}

void pwm1_init(void) {
    /* Timer 1 */
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10) | (1 << WGM11);
    TCCR1B = (1 << WGM12) | (1 << CS10) | (1 << CS12);
    DDRB |= (1 << PORTB1) | (1 << PORTB2);

    OCR1A = 24;
    OCR1B = 24;
}


void pwm2_init(void) {
    /* Timer 2 */
    TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20) | (1 << WGM21);
    TCCR2B = (1 << CS20) | (1 << CS21) | (1 << CS22);
    DDRB |= (1 << PORTB3);
    DDRD |= (1 << PORTD3);

    OCR2A = 24;;
    OCR2B = 24;;
}

void adc_init() {
    ADMUX |= (1 << REFS0);
    ADCSRA = (1 << ADEN);       /* Enable ADC */
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);       /* Set base freq prescale */
}

uint16_t adc_read(uint8_t ch) {

    if (ch > 7)
        return 0;

    ch &= 0b00000111;
    ADMUX = (ADMUX & 0xf8) | ch;        /* Channel selection */
    ADCSRA |= (1 << ADSC);              /* Start conversion */

    while (ADCSRA & (1 << ADSC));

    return (ADC);
}

uint8_t map2 (uint16_t level, uint8_t min, uint8_t max) {
    return ((level * (max - min)) / 1024 + min);
}

int main(void) {

    pwm0_init();
    pwm1_init();
    pwm2_init();
    adc_init();
    sei();

    while (1) {

        uint16_t l0 = adc_read(0);
        uint16_t l1 = adc_read(1);
        uint16_t l2 = adc_read(2);
        uint16_t l3 = adc_read(3);

        OCR2B = map2(l0, 12, 36);
        OCR0B = map2(l1, 10, 38);
        OCR0A = map2(l2, 10, 38);
        OCR1A = map2(l3, 14, 34);

        _delay_ms(10);
    }

}
/* EOF */
