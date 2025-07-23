#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void) {
    // Configure PORTC0 as input (IR sensor)
    DDRC &= ~(1 << PC0);      // Clear bit to make it input
    PORTC |= (1 << PC0);      // Enable pull-up resistor

    // Configure PORTD0 as output (Bulb/Relay/LED)
    DDRD |= (1 << PD0);       // Set PD0 as output
    PORTD &= ~(1 << PD0);     // Initially turn off the bulb/relay

    while (1) {
        if (!(PINC & (1 << PC0))) {  // If IR sensor detects (LOW)
            PORTD |= (1 << PD0);     // Turn ON bulb
        } else {
            PORTD &= ~(1 << PD0);    // Turn OFF bulb
        }

        _delay_ms(100);  // Debounce delay
    }
}
