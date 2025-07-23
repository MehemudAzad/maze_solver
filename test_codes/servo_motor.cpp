#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>

// Send one PWM pulse to move servo to a specific angle
void servo_pulse(uint8_t angle) {
    // Convert angle (0–180) to pulse width (1–2 ms)
    uint16_t pulse_width = 1000 + ((uint32_t)angle * 1000) / 180;  // in microseconds

    // Send HIGH pulse
    PORTD |= (1 << PD0);
    for (uint16_t i = 0; i < pulse_width; i++) {
        _delay_us(1);
    }

    // Send LOW pulse for the rest of the 20ms period
    PORTD &= ~(1 << PD0);
    for (uint16_t i = 0; i < (20000 - pulse_width); i++) {
        _delay_us(1);
    }
}

int main(void) {
    // Set PD0 as output
    DDRD |= (1 << PD0);
    PORTD &= ~(1 << PD0);  // Make sure it's LOW initially

    while (1) {
        // Move to 0°
        for (uint8_t i = 0; i < 50; i++) servo_pulse(0);

        // Move to 90°
        for (uint8_t i = 0; i < 50; i++) servo_pulse(90);

        // Move to 180°
        for (uint8_t i = 0; i < 50; i++) servo_pulse(180);
    }
}
