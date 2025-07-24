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

// === PID Control Variables ===
float kp = 1.5;                   // Proportional gain
float ki = 0.05;                  // Integral gain  
float kd = 0.8;                   // Derivative gain
float pid_error = 0.0;            // Current error
float previous_error = 0.0;       // Previous error for derivative
float integral_error = 0.0;       // Accumulated error for integral
float pid_output = 0.0;           // PID controller output
uint8_t pid_enabled = 0;          // PID control enable flag




//! sonar init
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
    return (uint16_t)(count )*2.3/10;
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
    return (uint16_t)(count)*2.3/10;
}

uint8_t ir_sensor_read(void) {
	return (PINC & (1 << IR_SENSOR_PIN)) ? 1 : 0;
}

// === PID Controller Functions ===
float calculate_pid(float error) {
    // Calculate PID components
    float proportional = kp * error;
    
    // Integral component with windup protection
    integral_error += error;
    if (integral_error > 50) integral_error = 50;
    if (integral_error < -50) integral_error = -50;
    float integral = ki * integral_error;
    
    // Derivative component
    float derivative = kd * (error - previous_error);
    previous_error = error;
    
    // Calculate total PID output
    float output = proportional + integral + derivative;
    
    // Limit output to reasonable range
    if (output > 50) output = 50;
    if (output < -50) output = -50;
    
    return output;
}

// PID wall centering function
void pid_wall_follow() {
    uint16_t left_distance = sonar_get_distance_cm();   // Left sensor
    uint16_t right_distance = sonar2_get_distance_cm(); // Right sensor
    
    // Handle invalid readings
    if (left_distance == 0xFFFF) left_distance = 100;
    if (right_distance == 0xFFFF) right_distance = 100;
    
    // Calculate error: positive = too close to left, negative = too close to right
    float error = (float)left_distance - (float)right_distance;
    
    // Store current error for debugging
    pid_error = error;
    
    // Calculate PID output
    pid_output = calculate_pid(error);
    
    // Apply PID correction to motor speeds
    int16_t left_speed = motor_speed - (int16_t)pid_output;
    int16_t right_speed = motor_speed + (int16_t)pid_output;
    
    // Ensure speeds are within valid range
    if (left_speed > 255) left_speed = 255;
    if (left_speed < 90) left_speed = 90;    // Minimum speed to keep motors running
    if (right_speed > 255) right_speed = 255;
    if (right_speed < 90) right_speed = 90;
    
    // Set motor speeds
    set_left_motor_speed((uint8_t)left_speed);
    set_right_motor_speed((uint8_t)right_speed);
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
    
    if (pid_enabled) {
        // Use PID for wall following
        pid_wall_follow();
    } else {
        // Original manual speed control
        set_right_motor_speed(motor_speed-20);
        set_left_motor_speed(motor_speed-20+10);
    }
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
    serial_string("PID: P(toggle PID), C(check PID status), D(distance readings)\n");
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
                case 'D': // Distance readings from both sensors
                    {
                        uint16_t left_dist = sonar_get_distance_cm();
                        uint16_t right_dist = sonar2_get_distance_cm();
                        serial_string("Left sensor: ");
                        serial_num(left_dist);
                        serial_string(" cm | Right sensor: ");
                        serial_num(right_dist);
                        serial_string(" cm\n");
                        
                        // Show difference for debugging
                        int16_t difference = (int16_t)left_dist - (int16_t)right_dist;
                        serial_string("Difference (L-R): ");
                        serial_num(difference);
                        serial_string(" cm\n");
                    }
                    break;
                
                case 'P': // Toggle PID control
                    pid_enabled = !pid_enabled;
                    if (pid_enabled) {
                        serial_string("PID wall following ENABLED\n");
                        // Reset PID variables
                        integral_error = 0.0;
                        previous_error = 0.0;
                        pid_error = 0.0;
                    } else {
                        serial_string("PID wall following DISABLED\n");
                    }
                    break;
                
                case 'C': // Check PID status and sensor readings
                    serial_string("PID Status: ");
                    if (pid_enabled) {
                        serial_string("ENABLED\n");
                        uint16_t left = sonar_get_distance_cm();
                        uint16_t right = sonar2_get_distance_cm();
                        serial_string("Left: ");
                        serial_num(left);
                        serial_string("cm | Right: ");
                        serial_num(right);
                        serial_string("cm\n");
                        serial_string("PID Error: ");
                        serial_num((int16_t)(pid_error * 10)); // Multiply by 10 for decimal display
                        serial_string(" | PID Output: ");
                        serial_num((int16_t)(pid_output * 10));
                        serial_string("\n");
                    } else {
                        serial_string("DISABLED\n");
                    }
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
                    serial_string("Available: F,B,L,R,S,I,D,P,C,0-9,a,t\n");
                    serial_string("F=Forward, B=Back, L=Left, R=Right, S=Stop\n");
                    serial_string("I=IR sensor, D=Distance sensors, P=PID toggle, C=PID check\n");
                    break;
            }
        }
    }
}