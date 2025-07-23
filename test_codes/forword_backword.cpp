#include <avr/io.h>
#define F_CPU 8000000UL  // 8MHz internal clock
#include <util/delay.h>

// Motor Pins - PORTD
#define IN1 PD0
#define IN2 PD1
#define ENA PD2
#define IN3 PD3
#define IN4 PD4
#define ENB PD5

// ===== Motor Control Functions =====
void all_forward() {
    PORTD |= (1 << IN1);
    PORTD &= ~(1 << IN2);
    PORTD |= (1 << ENA);

    PORTD |= (1 << IN3);
    PORTD &= ~(1 << IN4);
    PORTD |= (1 << ENB);
}

void all_backward() {
    PORTD &= ~(1 << IN1);
    PORTD |= (1 << IN2);
    PORTD |= (1 << ENA);

    PORTD &= ~(1 << IN3);
    PORTD |= (1 << IN4);
    PORTD |= (1 << ENB);
}

void all_stop() {
    PORTD &= ~(1 << ENA);
    PORTD &= ~(1 << ENB);
}

// ===== Main =====
int main(void) {
    // Set all motor control pins as output
    DDRD |= (1 << IN1) | (1 << IN2) | (1 << ENA) |
            (1 << IN3) | (1 << IN4) | (1 << ENB);

    while (1) {
        all_forward();
        _delay_ms(200);  // Move forward for 2 seconds

        all_backward();
        _delay_ms(200);  // Move backward for 2 seconds
    }
}

