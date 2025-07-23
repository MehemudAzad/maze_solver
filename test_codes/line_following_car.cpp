#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>

// Sensor pins
#define LEFT_SENSOR  PA0
#define RIGHT_SENSOR PA1

// Motor pins - L298N
// Motor Pins - PORTD
#define IN1 PD2
#define IN2 PD3
#define ENA PD4
#define IN3 PD5
#define IN4 PD6
#define ENB PD7

// ===== Function Prototypes =====
void initADC();
uint8_t readSensor(uint8_t channel);
void forward();
void backward_left();
void backward_right();
void stop_motors();

int main(void) {
    // Motor pins as output
    DDRD |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);

    // Sensor pins as input
    DDRA &= ~((1 << LEFT_SENSOR) | (1 << RIGHT_SENSOR));

    initADC();  // Initialize ADC

    while (1) {
        uint8_t left = readSensor(0);   // ADC0 = PA0
        uint8_t right = readSensor(1);  // ADC1 = PA1

        if (left < 100 && right < 100) {
            forward();
        } else if (left < 100 && right > 100) {
            backward_left();
        } else if (left > 100 && right < 100) {
            backward_right();
        } else {
            stop_motors();
        }

        _delay_ms(50);
    }
}

// ===== ADC Init & Read =====
void initADC() {
    ADMUX = (1 << REFS0); // AVcc reference
    ADCSRA = (1 << ADEN)  // Enable ADC
           | (1 << ADPS2) | (1 << ADPS1); // Prescaler = 64
}

uint8_t readSensor(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Select channel
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait
    return ADCH; // 8-bit result
}

// ===== Motor Control Functions =====
void forward() {
    // Both motors forward
    PORTD |= (1 << IN1);
    PORTD &= ~(1 << IN2);
    PORTD |= (1 << IN3);
    PORTD &= ~(1 << IN4);
}

void backward_left() {
    // Turn left: left backward, right forward
    PORTD &= ~(1 << IN1);
    PORTD |= (1 << IN2);
    PORTD |= (1 << IN3);
    PORTD &= ~(1 << IN4);
}

void backward_right() {
    // Turn right: left forward, right backward
    PORTD |= (1 << IN1);
    PORTD &= ~(1 << IN2);
    PORTD &= ~(1 << IN3);
    PORTD |= (1 << IN4);
}

void stop_motors() {
    PORTD &= ~((1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4));
}
