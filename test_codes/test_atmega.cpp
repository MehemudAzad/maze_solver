#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    // Set all pins of PORTA, PORTB, PORTC, and PORTD as output
    DDRA = 0xFF;  // Data Direction Register for Port A: all outputs
    DDRB = 0xFF;  // Port B
    DDRC = 0xFF;  // Port C
    DDRD = 0xFF;  // Port D

    while (1) {
        // Set all pins high (logic 1)
        PORTA = 0xFF;
        PORTB = 0xFF;
        PORTC = 0xFF;
        PORTD = 0xFF;

        _delay_ms(1000);  // Delay for 1 second

        // Optionally, set all pins low for testing
        PORTA = 0x00;
        PORTB = 0x00;
        PORTC = 0x00;
        PORTD = 0x00;

        _delay_ms(1000);
    }
}
