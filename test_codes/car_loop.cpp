#include <avr/io.h>
#define F_CPU 1000000UL // 1 MHz CPU clock
#include <util/delay.h>

// ==== LEFT MOTOR GROUP ====
#define IN1 PD2 // Direction Pin 1 for Left Motor
#define IN2 PD3 // Direction Pin 2 for Left Motor
// ENA (Enable) for Left Motor will be PD4 (OC1B) - controlled by PWM

// ==== RIGHT MOTOR GROUP ====
#define IN3 PD6 // Direction Pin 1 for Right Motor (adjusted from original PD5 to free up OC1A)
#define IN4 PD7 // Direction Pin 2 for Right Motor (adjusted from original PD6 to free up OC1A)
// ENB (Enable) for Right Motor will be PD5 (OC1A) - controlled by PWM


/*
 * Function: init_pwm_timer1
 * -------------------------
 * Initializes Timer1 for Fast PWM mode, controlling PD4 (OC1B) and PD5 (OC1A).
 *
 * PWM Frequency Calculation:
 * F_PWM = F_CPU / (Prescaler * (TOP + 1))
 *
 * With F_CPU = 1 MHz, Prescaler = 8, TOP = 255:
 * F_PWM = 1,000,000 / (8 * (255 + 1)) = 1,000,000 / (8 * 256) = 1,000,000 / 2048 = ~488 Hz
 *
 * A TOP of 255 gives 256 steps of speed control (0-255).
 * The prescaler of 8 gives a reasonable PWM frequency for motor control.
 *
 * This setup uses Non-Inverting Fast PWM mode (Mode 14 with ICR1 as TOP, but here using 0xFF / 255 as TOP).
 * For ATmega32, setting WGM13:0 to 1110 (Mode 14) implies TOP is in ICR1.
 * However, to use 8-bit resolution (0-255), we can use Mode 5, 6, or 7.
 * Let's use Mode 5: Fast PWM, 8-bit (TOP = 0xFF), which corresponds to WGM12=1, WGM10=1.
 *
 * For OC1A (PD5) and OC1B (PD4):
 * COM1A1:0 = 10 (Clear OC1A on Compare Match, Set at TOP) - Non-inverting
 * COM1B1:0 = 10 (Clear OC1B on Compare Match, Set at TOP) - Non-inverting
 */
void init_pwm_timer1() {
    // Set PD4 (OC1B) and PD5 (OC1A) as output pins for PWM
    DDRD |= (1 << PD4) | (1 << PD5);

    // Configure Timer1 Control Register A (TCCR1A)
    // WGM10: Set to 1 for Fast PWM (8-bit)
    // COM1A1: Set to 1 for non-inverting mode for OC1A (PD5)
    // COM1B1: Set to 1 for non-inverting mode for OC1B (PD4)
    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1);

    // Configure Timer1 Control Register B (TCCR1B)
    // WGM12: Set to 1 for Fast PWM (8-bit)
    // CS11: Set for Prescaler of 8 (F_CPU / 8)
    TCCR1B = (1 << WGM12) | (1 << CS11);

    // Initial duty cycle (0% speed)
    OCR1A = 0; // Duty cycle for OC1A (Right Motor Speed)
    OCR1B = 0; // Duty cycle for OC1B (Left Motor Speed)
}

/*
 * Function: set_left_motor_speed
 * ------------------------------
 * Sets the speed of the left motor.
 * speed: An integer value from 0 (off) to 255 (full speed).
 */
void set_left_motor_speed(uint8_t speed) {
    OCR1B = speed; // Set duty cycle for OC1B (PD4)
}

/*
 * Function: set_right_motor_speed
 * -------------------------------
 * Sets the speed of the right motor.
 * speed: An integer value from 0 (off) to 255 (full speed).
 */
void set_right_motor_speed(uint8_t speed) {
    OCR1A = speed; // Set duty cycle for OC1A (PD5)
}


// ==== Motor Control Functions with Speed Parameter ====

void all_forward(uint8_t speed) {
    // Left motor forward
    PORTD |= (1 << IN1);  // IN1 HIGH
    PORTD &= ~(1 << IN2); // IN2 LOW
    set_left_motor_speed(speed);

    // Right motor forward
    PORTD |= (1 << IN3);  // IN3 HIGH
    PORTD &= ~(1 << IN4); // IN4 LOW
    set_right_motor_speed(speed);
}

void all_backward(uint8_t speed) {
    // Left motor backward
    PORTD &= ~(1 << IN1); // IN1 LOW
    PORTD |= (1 << IN2);  // IN2 HIGH
    set_left_motor_speed(speed);

    // Right motor backward
    PORTD &= ~(1 << IN3); // IN3 LOW
    PORTD |= (1 << IN4);  // IN4 HIGH
    set_right_motor_speed(speed);
}

void turn_left_soft(uint8_t speed) {
    // Left motor stop or low speed for a softer turn
    // Option 1: Stop left motor completely
    set_left_motor_speed(0);
    // Option 2: Keep left motor at a reduced speed (e.g., speed / 2)
    // set_left_motor_speed(speed / 2);

    // Right motor forward at full specified speed
    PORTD |= (1 << IN3);  // IN3 HIGH
    PORTD &= ~(1 << IN4); // IN4 LOW
    set_right_motor_speed(speed);
}

void turn_right_soft(uint8_t speed) {
    // Left motor forward at full specified speed
    PORTD |= (1 << IN1);  // IN1 HIGH
    PORTD &= ~(1 << IN2); // IN2 LOW
    set_left_motor_speed(speed);

    // Right motor stop or low speed for a softer turn
    // Option 1: Stop right motor completely
    set_right_motor_speed(0);
    // Option 2: Keep right motor at a reduced speed (e.g., speed / 2)
    // set_right_motor_speed(speed / 2);
}

void all_stop() {
    // Set both motor speeds to 0
    set_left_motor_speed(0);
    set_right_motor_speed(0);

    // Optionally, set direction pins to low to ensure motors are truly off
    // or if the driver requires it. Most drivers with ENA/ENB don't strictly need this.
    // PORTD &= ~((1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4));
}


int main(void) {
    // Set motor direction pins as output
    DDRD |= (1 << IN1) | (1 << IN2) |
            (1 << IN3) | (1 << IN4);

    // Initialize Timer1 for PWM on PD4 and PD5
    init_pwm_timer1();

    // Define a base speed (0-255)
    // You can change this value to adjust the overall speed of the car.
    uint8_t current_speed = 80; // Example: ~60% of max speed (150/255)

    while (1) {
        // --- Movement Sequence ---

        all_forward(current_speed);
        _delay_ms(2500);      // Move forward

        turn_left_soft(current_speed);
        _delay_ms(1000);      // Turn left

        all_forward(current_speed);
        _delay_ms(2500);      // Move forward again

        turn_left_soft(current_speed); // Changed back to turn_right_soft based on the typical sequence
        _delay_ms(1000);      // Turn right

        all_forward(current_speed);
        _delay_ms(2500);

        turn_left_soft(current_speed); // Example of moving backward
        _delay_ms(1000);

        all_forward(current_speed);
        _delay_ms(2500);

        turn_left_soft(current_speed);
        _delay_ms(1000);

        all_stop();
        _delay_ms(4000);      // Stop for a longer duration

        // If you want the sequence to run only once, uncomment the break;
        // break;
    }
}