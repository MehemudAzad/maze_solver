#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "serial.h"
// #include "i2c.h"
// #include "mpu6050.h"
// Pin assignments
#define MOTOR1_IN1    PA0    // Motor 1 direction (Right side, L298N IN1)
#define MOTOR1_IN2    PA1    // Motor 1 direction (L298N IN2)
#define MOTOR2_IN1    PA2    // Motor 2 direction (Left side, L298N IN3)
#define MOTOR2_IN2    PA3    // Motor 2 direction (L298N IN4)
#define EN_MOTOR1     PB3    // Enable motor 1 (OC0 - Timer0, L298N ENA, Right motor)
#define EN_MOTOR2     PD7    // Enable motor 2 (OC2 - Timer2, L298N ENB, Left motor)
#define SONAR_TRIG PD2
#define SONAR_ECHO PD3

#define SONAR2_TRIG PD4
#define SONAR2_ECHO PD5

#define IR_SENSOR_PIN PC0 // Example pin for IR sensor
void motor_stop(void);
void motor_forward(void);
void motor_reverse(void);
void motor_left(void);
void motor_right(void);
void set_right_motor_speed(uint8_t speed);
void set_left_motor_speed(uint8_t speed);
// Speed control variables
uint8_t motor_speed = 120;  // Default speed (0-255)
uint8_t speed_increment = 25; // Speed change step
uint8_t control_mode = 0;   // 0 = Bluetooth, 1 = Gesture


void sonar_init(void) {
    DDRD |= (1 << SONAR_TRIG);  // TRIG as output
    DDRD &= ~(1 << SONAR_ECHO); // ECHO as input
}

void sonar2_init(void) {
    DDRD |= (1 << SONAR2_TRIG);  // TRIG2 as output
    DDRD &= ~(1 << SONAR2_ECHO); // ECHO2 as input
}

uint16_t sonar_get_distance_cm(void) {
    // Send 10us pulse to TRIG
    PORTD &= ~(1 << SONAR_TRIG);
    _delay_us(2);
    PORTD |= (1 << SONAR_TRIG);
    _delay_us(10);
    PORTD &= ~(1 << SONAR_TRIG);

    // Wait for ECHO to go high
    uint32_t timeout = 30000;
    while (!(PIND & (1 << SONAR_ECHO))) {
        if (--timeout == 0) return 0xFFFF; // Timeout
    }

    // Measure high time
    uint32_t count = 0;
    while (PIND & (1 << SONAR_ECHO)) {
        count++;
        _delay_us(1);
        if (count > 30000) return 0xFFFF; // Timeout
    }

    // Sound speed: 343 m/s, so distance (cm) = (time_us / 58)
    return (uint16_t)(count );
}

uint16_t sonar2_get_distance_cm(void) {
    PORTD &= ~(1 << SONAR2_TRIG);
    _delay_us(2);
    PORTD |= (1 << SONAR2_TRIG);
    _delay_us(10);
    PORTD &= ~(1 << SONAR2_TRIG);

    uint32_t timeout = 30000;
    while (!(PIND & (1 << SONAR2_ECHO))) {
        if (--timeout == 0) return 0xFFFF;
    }
    uint32_t count = 0;
    while (PIND & (1 << SONAR2_ECHO)) {
        count++;
        _delay_us(1);
        if (count > 30000) return 0xFFFF;
    }
    return (uint16_t)(count);
}

uint8_t ir_sensor_read(void) {
	return (PINC & (1 << IR_SENSOR_PIN)) ? 1 : 0;
}
void pwm_init() {
    DDRB |= (1 << EN_MOTOR1);  // PB3 (OC0) - Right motor ENA
    DDRD |= (1 << EN_MOTOR2);  // PD7 (OC2) - Left motor ENB

    // Timer0 - Fast PWM, Non-inverting, Prescaler 8
    TCCR0 = (1 << WGM00) | (1 << WGM01) | (1 << COM01) | (1 << CS01); // Prescaler 8 (~488 Hz at 1 MHz)
    
    // Timer2 - Fast PWM, Non-inverting, Prescaler 8
    TCCR2 = (1 << WGM20) | (1 << WGM21) | (1 << COM21) | (1 << CS21); // Prescaler 8 (~488 Hz at 1 MHz)

    OCR0 = motor_speed;    // Left motor (PD7, OC2)
    OCR2 = motor_speed;    // Right motor (PB3, OC0)
}

static void motor_init() {
    // Set motor direction pins as outputs (PA0-PA3)
    DDRA |= (1 << MOTOR1_IN1) | (1 << MOTOR1_IN2) | (1 << MOTOR2_IN1) | (1 << MOTOR2_IN2);
    // Initialize direction pins to low
    PORTA &= ~((1 << MOTOR1_IN1) | (1 << MOTOR1_IN2) | (1 << MOTOR2_IN1) | (1 << MOTOR2_IN2));
    // Initialize PWM
    pwm_init();
}

// Set speed for all motors (0-255)
void set_motor_speed(uint8_t speed) {
    motor_speed = speed;
    OCR2 = speed;  // Right motor (PB3, OC0)
    OCR0 = speed;  // Left motor (PD7, OC2)
}

// Individual motor speed control
void set_right_motor_speed(uint8_t speed) {
    OCR2 = speed;  // Right motor (PB3, OC0)
}

void set_left_motor_speed(uint8_t speed) {
    OCR0 = speed;  // Left motor (PD7, OC2)
}

// Debug function to test PWM
void test_pwm() {
    serial_string("Testing PWM...\n");
    for (uint8_t i = 0; i <= 255; i += 51) {
        serial_string("Setting PWM to: ");
        serial_num(i);
        serial_string("\n");
        OCR2 = i;  // Right motor
        OCR0 = i;  // Left motor
        _delay_ms(1000);
    }
    OCR2 = 0;
    OCR0 = 0;
    serial_string("PWM test complete\n");
}

// Motor direction control
void motor_forward() {
    PORTA &= ~((1 << MOTOR1_IN1) | (1 << MOTOR2_IN1));
    PORTA |= (1 << MOTOR1_IN2) | (1 << MOTOR2_IN2);
    // set_right_motor_speed(motor_speed);
	// set_left_motor_speed(motor_speed+16.5);
	set_right_motor_speed(motor_speed-20);
	set_left_motor_speed(motor_speed-20+10);
	_delay_ms(1000);
}

void motor_stop() {
    OCR2 = 0;  // Right motor
    OCR0 = 0;  // Left motor
}

void motor_reverse() {
	PORTA &= ~((1 << MOTOR1_IN2) | (1 << MOTOR2_IN2));
    PORTA |= (1 << MOTOR1_IN1) | (1 << MOTOR2_IN1);
    set_motor_speed(motor_speed);
	set_right_motor_speed(motor_speed-20);
	set_left_motor_speed(motor_speed+16.5-20);
	_delay_ms(1000);
}

void motor_right() {
    PORTA &= ~((1 << MOTOR1_IN1) | (1 << MOTOR2_IN2));
    PORTA |= (1 << MOTOR1_IN2) | (1 << MOTOR2_IN1);
    set_right_motor_speed(motor_speed-30); // Slower right motor
    set_left_motor_speed(motor_speed-30);      // Full speed left motor
	_delay_ms(400);
}

void motor_left() {
	PORTA &= ~((1 << MOTOR1_IN2) | (1 << MOTOR2_IN1));
    PORTA |= (1 << MOTOR1_IN1) | (1 << MOTOR2_IN2);
    set_right_motor_speed(motor_speed-30);     // Full speed right motor
    set_left_motor_speed(motor_speed-30);  // Slower left motor
    // _delay_ms(500);
}

// Stub for autonomous behavior
static void autonomous_mode(void) {
    motor_forward();
    _delay_ms(2000);
    motor_stop();
    _delay_ms(1000);
    motor_left();
    _delay_ms(1000);
    motor_stop();
    _delay_ms(1000);
}

int main(void) {
    char cmd;
    uint8_t auto_mode = 0;
    serial_init(9600);
    motor_init();
	sonar_init(); // Initialize sonar sensor
	sonar2_init();
    motor_stop();
    serial_string("Motor control with PWM speed ready!\n");
    serial_string("Commands: F(forward), B(reverse), L(left), R(right), S(stop)\n");
    serial_string("Speed: +(increase), -(decrease), 0-9(set speed level)\n");
    serial_string("G: Gesture mode toggle\n");
    serial_string("Current speed: ");
    serial_num(motor_speed);
    serial_string("\n");

    while (1) {
        // Gesture mode: read MPU6050 and control motors
        // if (control_mode == 1) {
        //     int16_t ax, ay, az, gx, gy, gz;
        //     float pitch, roll;
        //     mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz);
        //     mpu6050_get_angles(ax, ay, az, &pitch, &roll);
        //     motor_gesture_control(pitch, roll);
        //     _delay_ms(100); // Adjust as needed
        //     continue;
        // }
        if (serial_available()) {
            cmd = serial_read();

            // Stop car if IR sensor is triggered
            if (ir_sensor_read() == 0) {
                motor_stop();
                serial_string("IR sensor triggered: Stopping car\n");
                continue;
            }

            if (cmd == 'a') {
                auto_mode = 1;
                serial_string("Entering autonomous mode\n");
                continue;
            }
            if (cmd == 's' && auto_mode) {
                auto_mode = 0;
                serial_string("Exiting autonomous mode\n");
                motor_stop();
                continue;
            }
            if (cmd == 'G' || cmd == 'g') {
                control_mode = !control_mode;
                if (control_mode) serial_string("Gesture mode ON\n");
                else serial_string("Gesture mode OFF\n");
                motor_stop();
                continue;
            }
            if (auto_mode) {
                autonomous_mode();
                continue;
            }
            switch (cmd) {
                case 'F':
                    motor_forward();
                    serial_string("Moving forward at speed ");
                    serial_num(motor_speed);
                    serial_string("\n");
                    break;
                case 'S':
                    motor_stop();
                    serial_string("Stopping\n");
                    break;
                case 'L':
                    motor_left();
                    serial_string("Turning left at speed ");
                    serial_num(motor_speed);
                    serial_string("\n");
                    break;
                case 'R':
                    motor_right();
                    serial_string("Turning right at speed ");
                    serial_num(motor_speed);
                    serial_string("\n");
                    break;
                case 'B':
                    motor_reverse();
                    serial_string("Reversing at speed ");
                    serial_num(motor_speed);
                    serial_string("\n");
                    break;
				case 'I': // IR sensor value
                    serial_string("IR sensor value: ");
                    serial_num(ir_sensor_read());
                    serial_string("\n");
                    break;
				case 'D': // Distance
					uint16_t dit2 = sonar2_get_distance_cm();
                    uint16_t dist = sonar_get_distance_cm();
                    serial_string("Distance: ");
                    serial_num(dist);
                    serial_string(" cm\n");
					serial_string(" cm, Distance 2: ");
					serial_num(dit2);
					serial_string(" cm\n");
                    break;

				
                case '0': case '1': case '2': case '3': case '4':
                case '5': case '6': case '7': case '8': case '9':
                    motor_speed = (cmd - '0') * 25;
                    if (cmd == '9') motor_speed = 255;
                    serial_string("Speed set to level ");
                    serial_char(cmd);
                    serial_string(" (");
                    serial_num(motor_speed);
                    serial_string(")\n");
                    set_motor_speed(motor_speed);
                    break;
                case 't': case 'T':
                    test_pwm();
                    break;
                default:
                    serial_string("Unknown command\n");
                    serial_string("Available: F,B,L,R,S,+,-,0-9,a,t\n");
                    break;
            }
        }
    }
}