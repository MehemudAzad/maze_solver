#include <avr/io.h>
#define F_CPU 1000000UL
#include <util/delay.h>

// ==== LEFT MOTOR GROUP ====
#define IN1 PD2
#define IN2 PD3
#define ENA PD4

// ==== RIGHT MOTOR GROUP ====
#define IN3 PD5
#define IN4 PD6
#define ENB PD7

#define TRIG_PIN PC0
#define ECHO_PIN PC1

void sonar_init()
{
  DDRC |= (1 << TRIG_PIN);  // TRIG as output
  DDRC &= ~(1 << ECHO_PIN); // ECHO as input
}

uint16_t sonar_get_distance()
{
  uint16_t count;

  // Send 10µs trigger pulse
  PORTC &= ~(1 << TRIG_PIN);
  _delay_us(2);
  PORTC |= (1 << TRIG_PIN);
  _delay_us(10);
  PORTC &= ~(1 << TRIG_PIN);

  // Wait for ECHO to go HIGH
  while (!(PINC & (1 << ECHO_PIN)))
    ;

  // Start timer
  TCNT1 = 0;
  TCCR1B = (1 << CS11); // Prescaler 8 → 1 tick = 1µs if F_CPU = 8MHz

  // Wait for ECHO to go LOW
  while (PINC & (1 << ECHO_PIN))
    ;

  // Stop timer
  TCCR1B = 0;

  count = TCNT1;

  // Convert to cm: speed of sound = 343 m/s = 0.0343 cm/µs
  // So: distance = time / 58 (round-trip)
  return count / 58;
}

void all_forward() {
    // Left motor forward
    PORTD |= (1 << IN1);
    PORTD &= ~(1 << IN2);
    PORTD |= (1 << ENA);

    // Right motor forward
    PORTD |= (1 << IN3);
    PORTD &= ~(1 << IN4);
    PORTD |= (1 << ENB);
}

void all_backward() {
    // Left motor backward
    PORTD &= ~(1 << IN1);
    PORTD |= (1 << IN2);
    PORTD |= (1 << ENA);

    // Right motor backward
    PORTD &= ~(1 << IN3);
    PORTD |= (1 << IN4);
    PORTD |= (1 << ENB);
}

void turn_left_soft() {
    // Left motor stop
    PORTD &= ~(1 << ENA);

    // Right motor forward
    PORTD |= (1 << IN3);
    PORTD &= ~(1 << IN4);
    PORTD |= (1 << ENB);
}

void turn_right_soft() {
    // Left motor forward
    PORTD |= (1 << IN1);
    PORTD &= ~(1 << IN2);
    PORTD |= (1 << ENA);

    // Right motor stop
    PORTD &= ~(1 << ENB);
}

void all_stop() {
    PORTD &= ~(1 << ENA);
    PORTD &= ~(1 << ENB);
}

int main(void) {
    // Set motor control pins as output
    DDRD |= (1 << IN1) | (1 << IN2) | (1 << ENA) |
            (1 << IN3) | (1 << IN4) | (1 << ENB);
	sonar_init();

    while (1) {
        all_forward();
        _delay_ms(100);      // Move forward

		// uint16_t distance = sonar_get_distance();
		// if(distance < 1){
		// 	all_stop();
		// 	_delay_ms(1000);
		// }

        // turn_left_soft();
        // _delay_ms(730);      // Turn left

        // all_forward();
        // _delay_ms(600);      // Move forward again

        // // turn_right_soft();
        // turn_left_soft();
        // _delay_ms(730);      // Turn right


        // all_forward();
        // _delay_ms(600);
        
        // // all_backward();
        // turn_left_soft();
        // _delay_ms(730);      // Move backward

        // all_forward();
        // _delay_ms(600);


        // turn_left_soft();
        // _delay_ms(730);  

        // all_stop();
        // _delay_ms(4000);      // Stop

        // Stop repeating
        // break;
    }
}
