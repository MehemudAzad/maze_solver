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

#define IR_SENSOR_PIN PB0 // Example pin for IR sensor
void motor_stop(void);
void motor_forward(void);
void motor_reverse(void);
void motor_left(void);
void motor_right(void);
void set_right_motor_speed(uint8_t speed);
void set_left_motor_speed(uint8_t speed);
// Speed control variables
uint8_t motor_speed = 140;  // Default speed (0-255)
uint8_t speed_increment = 25; // Speed change step
uint8_t control_mode = 0;   // 0 = Bluetooth, 1 = Gesture


//! gyro scope code
#define MPU6050_ADDR 0x68
#define ACCEL_SCALE 16384.0 // �2g range, 16384 LSB/g

// 50 Hz PWM setup
#define PULSE_CENTER 188 // 1504 �s (~0�)
#define PULSE_RANGE 125  // �1004�2004 �s (~-90� to +90�)

// === TWI (I2C) ===
void TWI_Init() {
	TWSR = 0x00; // Status register
	TWBR = 32;   // Bit rate register (~100kHz at 1 MHz)
	TWCR = (1 << TWEN); // Enable TWI
}

void TWI_Start() {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

void TWI_Stop() {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
	_delay_us(100);
}

void TWI_Write(unsigned char data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

unsigned char TWI_Read_ACK() {
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

unsigned char TWI_Read_NACK() {
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

// === MPU6050 ===
void MPU6050_Init() {
	TWI_Start();
	TWI_Write(MPU6050_ADDR << 1); // Write mode
	TWI_Write(0x6B); // Power management register
	TWI_Write(0x00); // Wake up
	TWI_Stop();

	TWI_Start();
	TWI_Write(MPU6050_ADDR << 1);
	TWI_Write(0x1C); // Accelerometer config register
	TWI_Write(0x00); // �2g range
	TWI_Stop();
}

void MPU6050_ReadAccel(int16_t *accelY, int16_t *accelZ) {
	TWI_Start();
	TWI_Write(MPU6050_ADDR << 1); // Write mode
	TWI_Write(0x3D); // Start at ACCEL_YOUT_H
	TWI_Stop();

	TWI_Start();
	TWI_Write((MPU6050_ADDR << 1) | 1); // Read mode
	TWI_Read_ACK(); // Skip X high
	TWI_Read_ACK(); // Skip X low
	*accelY = ((int16_t)(TWI_Read_ACK() << 8) | TWI_Read_ACK()); // Y high, low
	*accelZ = ((int16_t)(TWI_Read_ACK() << 8) | TWI_Read_NACK()); // Z high, low
	TWI_Stop();
}


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
    // Configure IR sensor pin as input with pull-up
    DDRB &= ~(1 << IR_SENSOR_PIN);  // Set as input
    PORTB |= (1 << IR_SENSOR_PIN);  // Enable pull-up resistor
    
    // Read the sensor (0 = obstacle detected, 1 = no obstacle)
    return (PINB & (1 << IR_SENSOR_PIN)) ? 1 : 0;
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

// Motor direction control with IR sensor obstacle detection
void motor_forward() {
    PORTA &= ~((1 << MOTOR1_IN1) | (1 << MOTOR2_IN1));
    PORTA |= (1 << MOTOR1_IN2) | (1 << MOTOR2_IN2);
    set_right_motor_speed(motor_speed + 4);
    set_left_motor_speed(motor_speed);
}

// Safe motor forward with continuous IR sensor monitoring
void motor_forward_with_ir_check() {
    // Start moving forward
    motor_forward();
    
    serial_string("Moving forward with IR obstacle detection...\n");
    
    // Continue moving until obstacle detected or stopped manually
    while (1) {
        // Check IR sensor (assuming 0 = obstacle detected, 1 = no obstacle)
        if (ir_sensor_read() == 0) {
            motor_stop();
            serial_string("*** OBSTACLE DETECTED! Car stopped by IR sensor ***\n");
            break;
        }
        
        // Small delay for sensor reading
        _delay_ms(50);
        
        // Check if there's a new command to stop
        if (serial_available()) {
            char cmd = serial_read();
            if (cmd == 'S' || cmd == 's') {
                motor_stop();
                serial_string("Manual stop command received\n");
                break;
            }
        }
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
	  set_right_motor_speed(motor_speed);
	  set_left_motor_speed(motor_speed+16.5);
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

// Enhanced autonomous mode with IR sensor obstacle detection
static void autonomous_mode(void) {
    serial_string("=== AUTONOMOUS MODE WITH IR OBSTACLE DETECTION ===\n");
    
    while (1) {
        // Move forward with IR sensor monitoring
        serial_string("Phase 1: Moving forward...\n");
        motor_forward_with_ir_check();

        _delay_ms(500); // Brief pause before checking for obstacles
        // If we reach here, obstacle was detected
        // serial_string("Phase 2: Obstacle detected, backing up...\n");
        // motor_reverse();
        // _delay_ms(1000);  // Back up for 1 second
        // motor_stop();

        // _delay_ms(500); // Pause before turning
        
        serial_string("Phase 3: Turning to avoid obstacle...\n");
        motor_right();
        _delay_ms(800);   // Turn right for 0.8 seconds
        motor_stop();
        
        serial_string("Phase 4: Brief pause before continuing...\n");
        _delay_ms(500);   // Brief pause
        
        // Check if user wants to exit autonomous mode
        if (serial_available()) {
            char cmd = serial_read();
            if (cmd == 's' || cmd == 'S') {
                serial_string("Exiting autonomous mode\n");
                motor_stop();
                break;
            }
        }
        
        // Continue the loop - will start moving forward again
    }
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
    serial_string("Commands:\n");
    serial_string("F - Forward with IR obstacle detection\n");
    serial_string("f - Forward without IR detection\n");
    serial_string("B - Reverse, L - Left, R - Right, S - Stop\n");
    serial_string("a - Autonomous mode with IR obstacle avoidance\n");
    serial_string("I - Check IR sensor, D - Check distance sensors\n");
    serial_string("0-9 - Set speed level, T - Test PWM\n");
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

            if (cmd == 'a') {
                auto_mode = 1;
                serial_string("Entering autonomous mode with IR obstacle detection\n");
                autonomous_mode();  // Call autonomous mode directly
                auto_mode = 0;      // Reset when done
                continue;
            }
            if (cmd == 'G' || cmd == 'g') {
                control_mode = !control_mode;
                if (control_mode) serial_string("Gesture mode ON\n");
                else serial_string("Gesture mode OFF\n");
                motor_stop();
                continue;
            }
            switch (cmd) {
                case 'F':
                    motor_forward_with_ir_check();  // Use IR-protected forward
                    break;
                case 'f':  // Lowercase 'f' for normal forward without IR check
                    motor_forward();
                    serial_string("Moving forward (IR check disabled) at speed ");
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