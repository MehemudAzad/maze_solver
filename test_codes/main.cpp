#include <avr/io.h>
#define F_CPU 8000000UL  // 8MHz internal clock
#include <util/delay.h>

// Motor Pins - PORTD
#define IN1 PD2
#define IN2 PD3
#define ENA PD4
#define IN3 PD5
#define IN4 PD6
#define ENB PD7

// USART Definitions
#define BAUD 9600
#define MYUBRR ((F_CPU / (16UL * BAUD)) - 1)

// ===== USART Setup =====
void USART_Init(unsigned int ubrr) {
    UBRRH = (unsigned char)(ubrr >> 8);
    UBRRL = (unsigned char)ubrr;
    UCSRB = (1 << RXEN) | (1 << TXEN); // Enable RX & TX
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0); // 8-bit data
}

char USART_Receive(void) {
    while (!(UCSRA & (1 << RXC)));  // Wait for data
    return UDR;
}

void USART_Transmit(char data) {
    while (!(UCSRA & (1 << UDRE))); // Wait until buffer is empty
    UDR = data;
}

void USART_SendString(const char* str) {
    while (*str) {
        USART_Transmit(*str++);
    }
}

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

void turn_left_soft() {
    PORTD &= ~(1 << ENA);  // Stop left
    PORTD |= (1 << IN3);
    PORTD &= ~(1 << IN4);
    PORTD |= (1 << ENB);   // Right forward
}

void turn_right_soft() {
    PORTD |= (1 << IN1);
    PORTD &= ~(1 << IN2);
    PORTD |= (1 << ENA);   // Left forward
    PORTD &= ~(1 << ENB);  // Stop right
}

void all_stop() {
    PORTD &= ~(1 << ENA);
    PORTD &= ~(1 << ENB);
}

// ===== Main =====
int main(void) {
    DDRD |= (1 << IN1) | (1 << IN2) | (1 << ENA) |
            (1 << IN3) | (1 << IN4) | (1 << ENB);

    USART_Init(MYUBRR);  // Start USART

    while (1) {
        char command = USART_Receive();  // Wait for Bluetooth command

        // Ignore newline / carriage return characters
        if (command == '\n' || command == '\r') {
            continue;
        }

        // Debug: show received character in brackets
        USART_Transmit('[');
        USART_Transmit(command);
        USART_Transmit(']');

        switch (command) {
            case 'A':
            case 'a':
                USART_SendString(" Forward\n");
                all_forward();
                break;

            case 'B':
            case 'b':
                USART_SendString(" Backward\n");
                all_backward();
                break;

            case 'L':
            case 'l':
                USART_SendString(" Left\n");
                turn_left_soft();
                break;

            case 'R':
            case 'r':
                USART_SendString(" Right\n");
                turn_right_soft();
                break;

            case 'S':
            case 's':
                USART_SendString(" Stop\n");
                all_stop();
                break;

            default:
                USART_SendString(" Unknown command: ");
                USART_Transmit(command);
                USART_Transmit('\n');
                all_stop();
                break;
        }

        _delay_ms(100);  // Small delay for stability
    }
}
