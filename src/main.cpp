#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "serial.h"

// ==== MOTOR DEFINITIONS ====
#define MOTOR1_IN1 PA0  // Left motor direction 1
#define MOTOR1_IN2 PA1  // Left motor direction 2
#define MOTOR2_IN1 PA2  // Right motor direction 1
#define MOTOR2_IN2 PA3  // Right motor direction 2

// Motor speed control pins (PWM)
#define LEFT_MOTOR_PWM PD7   // OC2 - Timer2
#define RIGHT_MOTOR_PWM PB3  // OC0 - Timer0

#define SONAR_TRIG PD2
#define SONAR_ECHO PD3

#define SONAR2_TRIG PD4
#define SONAR2_ECHO PD5


// Arduino-style control variables
uint8_t motor_speed = 200;        // Base motor speed
uint8_t maxSpeed = 255;           // Maximum PWM value
uint8_t minSpeed = 160;           // Minimum PWM value at which motor moves
uint8_t equilibriumSpeed = 200;   // Rough estimate for straight driving
uint8_t leftSpeedVal = 200;
uint8_t rightSpeedVal = 200;
float targetAngle = 0.0;          // Target angle for precise turns
uint8_t isDriving = 0;            // 0 = rotating/stationary, 1 = driving forward
uint8_t prevIsDriving = 1;        // Previous driving state

// === PID Control Variables ===
float kp = 2.0;                   // Proportional gain
float ki = 0.1;                   // Integral gain  
float kd = 1.0;                   // Derivative gain
float pid_error = 0.0;            // Current error
float previous_error = 0.0;       // Previous error for derivative
float integral_error = 0.0;       // Accumulated error for integral
float pid_output = 0.0;           // PID controller output
uint16_t target_distance = 15;    // Target distance from wall (cm)
uint16_t max_sensor_distance = 50; // Maximum reliable sensor distance (cm)

// Wall following modes
#define WALL_FOLLOW_LEFT 1
#define WALL_FOLLOW_RIGHT 2  
#define WALL_FOLLOW_CENTER 3
uint8_t wall_follow_mode = WALL_FOLLOW_CENTER;

#define TRIG_PIN PC0
#define ECHO_PIN PC1

//! MPU
#define MPU6050_ADDR 0x68
#define ACCEL_SCALE 16384.0 // �2g range, 16384 LSB/g

// 50 Hz PWM setup
#define PULSE_CENTER 188 // 1504 �s (~0�)
#define PULSE_RANGE 125  // �1004�2004 �s (~-90� to +90�)

// === PWM Initialization ===
void pwm_init() {
    // Configure Timer0 for right motor PWM (PB3/OC0)
    DDRB |= (1 << PB3);  // Set PB3 as output
    TCCR0 = (1 << WGM00) | (1 << WGM01) | (1 << COM01) | (1 << CS01); // Fast PWM, non-inverting, prescaler 8
    OCR0 = 0; // Initial duty cycle
    
    // Configure Timer2 for left motor PWM (PD7/OC2)
    DDRD |= (1 << PD7);  // Set PD7 as output
    TCCR2 = (1 << WGM20) | (1 << WGM21) | (1 << COM21) | (1 << CS21); // Fast PWM, non-inverting, prescaler 8
    OCR2 = 0; // Initial duty cycle
}

//! sonar sensors
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





//!
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

void MPU6050_ReadAccel(int16_t *accelX, int16_t *accelY, int16_t *accelZ) {
	TWI_Start();
	TWI_Write(MPU6050_ADDR << 1); // Write mode
	TWI_Write(0x3B); // Start at ACCEL_XOUT_H, the first accelerometer register
	TWI_Stop();

	TWI_Start();
	TWI_Write((MPU6050_ADDR << 1) | 1); // Read mode
	// Read all 6 registers for X, Y, and Z
	*accelX = ((int16_t)(TWI_Read_ACK() << 8) | TWI_Read_ACK());
	*accelY = ((int16_t)(TWI_Read_ACK() << 8) | TWI_Read_ACK());
	*accelZ = ((int16_t)(TWI_Read_ACK() << 8) | TWI_Read_NACK());
	TWI_Stop();
}

void MPU6050_ReadGyro(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ) {
	TWI_Start();
	TWI_Write(MPU6050_ADDR << 1); // Write mode
	TWI_Write(0x43); // Start at GYRO_XOUT_H
	TWI_Stop();

	TWI_Start();
	TWI_Write((MPU6050_ADDR << 1) | 1); // Read mode
	// Read all 6 registers for X, Y, and Z gyroscope
	*gyroX = ((int16_t)(TWI_Read_ACK() << 8) | TWI_Read_ACK());
	*gyroY = ((int16_t)(TWI_Read_ACK() << 8) | TWI_Read_ACK());
	*gyroZ = ((int16_t)(TWI_Read_ACK() << 8) | TWI_Read_NACK());
	TWI_Stop();
}

// // === Servo PWM ===
// void Servo_PWM_Init() {
// 	DDRD |= (1 << PD5); // PD5 (OC1A) as output
// 	ICR1 = 2500; // 20 ms period (50 Hz) at 1 MHz / 8
// 	TCCR1A = (1 << COM1A1) | (1 << WGM11); // Fast PWM, non-inverting
// 	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Fast PWM mode 14, prescaler 8
// 	OCR1A = PULSE_CENTER; // Center position (0�)
// }
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
    OCR0 = speed;  // Right motor (PB3, OC0)
    OCR2 = speed;  // Left motor (PD7, OC2)
}

// Individual motor speed control
void set_right_motor_speed(uint8_t speed) {
    OCR0 = speed;  // Right motor (PB3, OC0)
}

void set_left_motor_speed(uint8_t speed) {
    OCR2 = speed;  // Left motor (PD7, OC2)
}

// Arduino-style motor control functions
void stopCar() {
    PORTA &= ~((1 << MOTOR1_IN1) | (1 << MOTOR1_IN2) | (1 << MOTOR2_IN1) | (1 << MOTOR2_IN2));
    OCR0 = 0;  // Right motor
    OCR2 = 0;  // Left motor
}

void motor_forward() {
    // Set motor directions for forward movement
    PORTA &= ~((1 << MOTOR1_IN1) | (1 << MOTOR2_IN1));
    PORTA |= (1 << MOTOR1_IN2) | (1 << MOTOR2_IN2);
}

void motor_left_turn() {
    // Left motor backward, right motor forward (spot turn left)
    PORTA &= ~((1 << MOTOR1_IN2) | (1 << MOTOR2_IN1));
    PORTA |= (1 << MOTOR1_IN1) | (1 << MOTOR2_IN2);
}

void motor_right_turn() {
    // Left motor forward, right motor backward (spot turn right)
    PORTA &= ~((1 << MOTOR1_IN1) | (1 << MOTOR2_IN2));
    PORTA |= (1 << MOTOR1_IN2) | (1 << MOTOR2_IN1);
}

// Arduino-style speed adjustment function
uint8_t changeSpeed(uint8_t motorSpeed, int8_t increment) {
    int16_t newSpeed = motorSpeed + increment;
    if (newSpeed > maxSpeed) {
        newSpeed = maxSpeed;
    } else if (newSpeed < minSpeed) {
        newSpeed = minSpeed;
    }
    return (uint8_t)newSpeed;
}

// === PID Controller Functions ===
float calculate_pid(float error) {
    // Calculate PID components
    float proportional = kp * error;
    
    integral_error += error;
    // Prevent integral windup
    if (integral_error > 100) integral_error = 100;
    if (integral_error < -100) integral_error = -100;
    float integral = ki * integral_error;
    
    float derivative = kd * (error - previous_error);
    previous_error = error;
    
    // Calculate total PID output
    float output = proportional + integral + derivative;
    
    // Limit output to reasonable range
    if (output > 100) output = 100;
    if (output < -100) output = -100;
    
    return output;
}

// Wall following with PID control
void wall_follow_pid() {
    uint16_t left_distance = sonar_get_distance_cm();   // Left sensor
    uint16_t right_distance = sonar2_get_distance_cm(); // Right sensor
    
    // Filter out invalid readings
    if (left_distance > max_sensor_distance) left_distance = max_sensor_distance;
    if (right_distance > max_sensor_distance) right_distance = max_sensor_distance;
    
    float error = 0.0;
    
    switch (wall_follow_mode) {
        case WALL_FOLLOW_LEFT:
            // Follow left wall at target distance
            error = target_distance - left_distance;
            break;
            
        case WALL_FOLLOW_RIGHT:
            // Follow right wall at target distance  
            error = right_distance - target_distance;
            break;
            
        case WALL_FOLLOW_CENTER:
            // Stay centered between walls
            error = (float)left_distance - (float)right_distance;
            break;
    }
    
    // Calculate PID output
    pid_output = calculate_pid(error);
    
    // Apply PID correction to motor speeds
    int16_t base_speed = equilibriumSpeed;
    int16_t left_correction = (int16_t)(pid_output);
    int16_t right_correction = (int16_t)(-pid_output);
    
    // Calculate new motor speeds
    int16_t new_left_speed = base_speed + left_correction;
    int16_t new_right_speed = base_speed + right_correction;
    
    // Ensure speeds are within valid range
    if (new_left_speed > maxSpeed) new_left_speed = maxSpeed;
    if (new_left_speed < minSpeed) new_left_speed = minSpeed;
    if (new_right_speed > maxSpeed) new_right_speed = maxSpeed;
    if (new_right_speed < minSpeed) new_right_speed = minSpeed;
    
    // Set motor speeds
    leftSpeedVal = (uint8_t)new_left_speed;
    rightSpeedVal = (uint8_t)new_right_speed;
    
    // Apply motor directions and speeds
    motor_forward();
    set_left_motor_speed(leftSpeedVal);
    set_right_motor_speed(rightSpeedVal);
}

// Arduino-style precise rotation control
void rotate_to_target(float currentAngle, float gyroX) {
    int16_t deltaAngle = (int16_t)(targetAngle - currentAngle);
    int16_t targetGyroX;
    
    if (abs(deltaAngle) <= 1) {
        stopCar();
    } else {
        if (currentAngle > targetAngle) { // turn left
            motor_left_turn();
        } else if (currentAngle < targetAngle) { // turn right
            motor_right_turn();
        }
        
        // Proportional control - same as Arduino
        if (abs(deltaAngle) > 30) {
            targetGyroX = 60;
        } else {
            targetGyroX = 2 * abs(deltaAngle);
        }
        
        // Speed adjustment based on gyroscope feedback
        if ((int16_t)(targetGyroX - abs(gyroX)) == 0) {
            // No adjustment needed
        } else if (targetGyroX > abs(gyroX)) {
            leftSpeedVal = changeSpeed(leftSpeedVal, 1); // Increase speed
        } else {
            leftSpeedVal = changeSpeed(leftSpeedVal, -1); // Decrease speed
        }
        
        rightSpeedVal = leftSpeedVal;
        set_right_motor_speed(rightSpeedVal);
        set_left_motor_speed(leftSpeedVal);
    }
}

// Arduino-style driving control with PID wall following
void driving_control(float currentAngle, float gyroX) {
    int16_t deltaAngle = (int16_t)(targetAngle - currentAngle);
    
    // If we're close to target angle, use PID wall following
    if (abs(deltaAngle) < 5) {
        wall_follow_pid();
    } else {
        // Use original angle-based steering for larger corrections
        motor_forward();
        
        if (deltaAngle != 0) {
            int16_t targetGyroX;
            
            // Proportional control for steering while driving
            if (deltaAngle > 30) {
                targetGyroX = 60;
            } else if (deltaAngle < -30) {
                targetGyroX = -60;
            } else {
                targetGyroX = 2 * deltaAngle;
            }
            
            // Adjust left motor speed for steering
            if ((int16_t)(targetGyroX - gyroX) == 0) {
                // No adjustment needed
            } else if (targetGyroX > gyroX) {
                leftSpeedVal = changeSpeed(leftSpeedVal, -1);
            } else {
                leftSpeedVal = changeSpeed(leftSpeedVal, 1);
            }
            
            rightSpeedVal = maxSpeed;
            set_right_motor_speed(rightSpeedVal);
            set_left_motor_speed(leftSpeedVal);
        }
    }
}

// Maze navigation decision making
uint8_t check_maze_decisions() {
    uint16_t left_distance = sonar_get_distance_cm();
    uint16_t right_distance = sonar2_get_distance_cm();
    
    // Define threshold for detecting openings
    uint16_t opening_threshold = 25; // cm
    
    uint8_t left_open = (left_distance > opening_threshold);
    uint8_t right_open = (right_distance > opening_threshold);
    
    // Decision logic for maze solving (right-hand rule)
    if (right_open) {
        return 1; // Turn right
    } else if (!left_open && !right_open) {
        return 2; // Dead end - turn around
    } else if (left_open) {
        return 3; // Turn left
    }
    
    return 0; // Continue forward
}

// Debug function to test PWM
void test_pwm() {
    serial_string("Testing PWM...\n");
    for (uint8_t i = 0; i <= 255; i += 51) {
        serial_string("Setting PWM to: ");
        serial_num(i);
        serial_string("\n");
        set_right_motor_speed(i);
        set_left_motor_speed(i);
        _delay_ms(1000);
    }
    set_right_motor_speed(0);
    set_left_motor_speed(0);
    serial_string("PWM test complete\n");
}


int main() {
    serial_init(9600);
    
    // Initialize LED pins (PB0, PB1) as outputs
    DDRB |= (1 << PB0) | (1 << PB1);
    PORTB &= ~((1 << PB0) | (1 << PB1)); // Initially turn off LEDs
    
    // Initialize motor system
    motor_init();
    
    // Initialize sonar sensors
    sonar_init();
    sonar2_init();
    
    TWI_Init();
    MPU6050_Init();

    // Raw sensor data
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    
    // Converted accelerometer and gyroscope values
    float AccX, AccY, AccZ;
    float GyroX, GyroY, GyroZ;
    
    // Angle calculations
    float accAngleX, accAngleY;
    float gyroAngleX = 0, gyroAngleY = 0, yaw = 0;
    float roll = 0, pitch = 0;
    float angle = 0; // Main angle used for navigation (equivalent to Arduino's angle)
    
    // Error compensation values
    float AccErrorX = 0, AccErrorY = 0;
    float GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
    
    // Timing for gyroscope integration
    uint32_t currentTime_ms, previousTime_ms, elapsedTime_ms;
    float elapsedTime_sec;
    
    char cmd;
    uint8_t calibrated = 0;
    uint8_t paused = 0;
    
    // Arduino-style control variables
    static uint8_t count = 0;
    static uint8_t countStraight = 0;

    serial_string("MPU6050 Maze Solver Car Ready\n");
    serial_string("Commands:\n");
    serial_string("'C' - Calibrate (keep stationary!)\n");
    serial_string("'W' - Drive forward with PID wall following\n");
    serial_string("'A' - Turn left 90 degrees\n");
    serial_string("'D' - Turn right 90 degrees\n");
    serial_string("'Q' - Stop/brake\n");
    serial_string("'I' - Print sensor information\n");
    serial_string("'P' - Pause/unpause\n");
    serial_string("'M' - Auto maze solving mode\n");
    serial_string("'L' - Follow left wall\n");
    serial_string("'R' - Follow right wall\n");
    serial_string("'B' - Follow center between walls\n");
    serial_string("'S' - Print sonar readings\n");

    // Initialize timing
    currentTime_ms = 0;
    previousTime_ms = 0;
    
    // Initialize speed values
    leftSpeedVal = equilibriumSpeed;
    rightSpeedVal = equilibriumSpeed;

    while (1) {
        // Read raw sensor data
        MPU6050_ReadAccel(&accelX, &accelY, &accelZ);
        MPU6050_ReadGyro(&gyroX, &gyroY, &gyroZ);
        
        // Convert to proper units
        AccX = accelX / 16384.0;  // Convert to g (±2g range)
        AccY = accelY / 16384.0;
        AccZ = accelZ / 16384.0;
        
        GyroX = gyroX / 131.0;    // Convert to degrees/second (±250°/s range)
        GyroY = gyroY / 131.0;
        GyroZ = gyroZ / 131.0;
        
        if (calibrated) {
            // Apply error compensation
            GyroX -= GyroErrorX;
            GyroY -= GyroErrorY;
            GyroZ -= GyroErrorZ;
            
            // Timing calculation (Arduino-style)
            previousTime_ms = currentTime_ms;
            currentTime_ms += 50; // Approximate 50ms loop time
            elapsedTime_ms = currentTime_ms - previousTime_ms;
            elapsedTime_sec = elapsedTime_ms / 1000.0;
            
            // Calculate angles from accelerometer
            accAngleX = (atan(AccY / sqrt(AccX*AccX + AccZ*AccZ)) * 180.0 / M_PI) - AccErrorX;
            accAngleY = (atan(-AccX / sqrt(AccY*AccY + AccZ*AccZ)) * 180.0 / M_PI) - AccErrorY;
            
            // Integrate gyroscope data
            gyroAngleX += GyroX * elapsedTime_sec;
            gyroAngleY += GyroY * elapsedTime_sec;
            yaw += GyroZ * elapsedTime_sec;
            
            // Complementary filter (same as Arduino: 96% gyro + 4% accelerometer)
            roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
            pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
            angle = roll; // Main navigation angle
        }
        
        // Check for bluetooth commands (Arduino-style command handling)
        if (serial_available()) {
            cmd = serial_read();
            
            if (cmd == 'W' || cmd == 'w') { // drive forward
                serial_string("forward\n");
                isDriving = 1;
                PORTB |= (1 << PB0); // Turn on LED to indicate driving
                
            } else if (cmd == 'A' || cmd == 'a') { // turn left
                serial_string("left\n");
                targetAngle += 90;
                if (targetAngle > 180) {
                    targetAngle -= 360;
                }
                isDriving = 0;
                PORTB |= (1 << PB1); // Turn on second LED for turning
                
            } else if (cmd == 'D' || cmd == 'd') { // turn right
                serial_string("right\n");
                targetAngle -= 90;
                if (targetAngle <= -180) {
                    targetAngle += 360;
                }
                isDriving = 0;
                PORTB |= (1 << PB1); // Turn on second LED for turning
                
            } else if (cmd == 'Q' || cmd == 'q') { // stop or brake
                serial_string("stop\n");
                isDriving = 0;
                stopCar();
                PORTB &= ~((1 << PB0) | (1 << PB1)); // Turn off LEDs
                
            } else if (cmd == 'I' || cmd == 'i') { // print information
                serial_string("Angle: ");
                serial_num((int16_t)angle);
                serial_string(" | Target: ");
                serial_num((int16_t)targetAngle);
                serial_string(" | GyroX: ");
                serial_num((int16_t)GyroX);
                serial_string(" | ElapsedTime(ms): ");
                serial_num((int16_t)(elapsedTime_sec * 1000));
                serial_string(" | EquilibriumSpeed: ");
                serial_num(equilibriumSpeed);
                serial_string("\n");
                
            } else if (cmd == 'P' || cmd == 'p') { // pause program
                paused = !paused;
                stopCar();
                isDriving = 0;
                serial_string("Program paused/unpaused\n");
                
            } else if (cmd == 'C' || cmd == 'c') {
                serial_string("Calibrating... Keep sensor stationary!\n");
                
                // Reset error values
                AccErrorX = 0; AccErrorY = 0;
                GyroErrorX = 0; GyroErrorY = 0; GyroErrorZ = 0;
                
                // Calibration with 200 samples
                for (uint16_t i = 0; i < 200; i++) {
                    MPU6050_ReadAccel(&accelX, &accelY, &accelZ);
                    MPU6050_ReadGyro(&gyroX, &gyroY, &gyroZ);
                    
                    // Convert to proper units
                    AccX = accelX / 16384.0;
                    AccY = accelY / 16384.0;
                    AccZ = accelZ / 16384.0;
                    GyroX = gyroX / 131.0;
                    GyroY = gyroY / 131.0;
                    GyroZ = gyroZ / 131.0;
                    
                    // Sum accelerometer angles
                    AccErrorX += (atan(AccY / sqrt(AccX*AccX + AccZ*AccZ)) * 180.0 / M_PI);
                    AccErrorY += (atan(-AccX / sqrt(AccY*AccY + AccZ*AccZ)) * 180.0 / M_PI);
                    
                    // Sum gyroscope readings
                    GyroErrorX += GyroX;
                    GyroErrorY += GyroY;
                    GyroErrorZ += GyroZ;
                    
                    _delay_ms(3);
                }
                
                // Calculate average errors
                AccErrorX = AccErrorX / 200.0;
                AccErrorY = AccErrorY / 200.0;
                GyroErrorX = GyroErrorX / 200.0;
                GyroErrorY = GyroErrorY / 200.0;
                GyroErrorZ = GyroErrorZ / 200.0;
                
                // Reset angle tracking
                gyroAngleX = 0; gyroAngleY = 0; yaw = 0;
                roll = 0; pitch = 0; angle = 0;
                targetAngle = 0; // Reset target angle
                currentTime_ms = 0;
                previousTime_ms = 0;
                
                // Reset PID variables
                pid_error = 0.0;
                previous_error = 0.0;
                integral_error = 0.0;
                
                calibrated = 1;
                serial_string("Calibration complete!\n");
                
            } else if (cmd == 'M' || cmd == 'm') { // Auto maze solving mode
                serial_string("Auto maze solving mode activated\n");
                isDriving = 1;
                wall_follow_mode = WALL_FOLLOW_RIGHT; // Use right-hand rule
                PORTB |= (1 << PB0); // Turn on driving LED
                
            } else if (cmd == 'L' || cmd == 'l') { // Follow left wall
                serial_string("Left wall following mode\n");
                wall_follow_mode = WALL_FOLLOW_LEFT;
                isDriving = 1;
                PORTB |= (1 << PB0);
                
            } else if (cmd == 'R' || cmd == 'r') { // Follow right wall
                serial_string("Right wall following mode\n");
                wall_follow_mode = WALL_FOLLOW_RIGHT;
                isDriving = 1;
                PORTB |= (1 << PB0);
                
            } else if (cmd == 'B' || cmd == 'b') { // Follow center between walls
                serial_string("Center wall following mode\n");
                wall_follow_mode = WALL_FOLLOW_CENTER;
                isDriving = 1;
                PORTB |= (1 << PB0);
                
            } else if (cmd == 'S' || cmd == 's') { // Print sonar readings
                uint16_t left_dist = sonar_get_distance_cm();
                uint16_t right_dist = sonar2_get_distance_cm();
                serial_string("Left: ");
                serial_num(left_dist);
                serial_string("cm | Right: ");
                serial_num(right_dist);
                serial_string("cm | PID Error: ");
                serial_num((int16_t)(pid_error * 10)); // Multiply by 10 for decimal display
                serial_string(" | PID Output: ");
                serial_num((int16_t)(pid_output * 10));
                serial_string(" | Mode: ");
                serial_num(wall_follow_mode);
                serial_string("\n");
            }
        }
        
        // Arduino-style main control loop (runs every ~50ms like Arduino's 20Hz)
        if (count < 6) {
            count++;
        } else { // runs once after loop runs 7 times
            count = 0;
            if (!paused && calibrated) {
                if (isDriving != prevIsDriving) {
                    leftSpeedVal = equilibriumSpeed;
                    countStraight = 0;
                    serial_string("Mode changed, isDriving: ");
                    serial_num(isDriving);
                    serial_string("\n");
                }
                
                if (isDriving) {
                    if (abs((int16_t)(targetAngle - angle)) < 3) {
                        if (countStraight < 20) {
                            countStraight++;
                        } else {
                            countStraight = 0;
                            equilibriumSpeed = leftSpeedVal; // Update equilibrium speed
                            serial_string("EQUILIBRIUM reached, speed: ");
                            serial_num(equilibriumSpeed);
                            serial_string("\n");
                        }
                    } else {
                        countStraight = 0;
                    }
                    driving_control(angle, GyroX);
                } else {
                    rotate_to_target(angle, GyroX);
                }
                prevIsDriving = isDriving;
            }
        }
    
        _delay_ms(5); // Maintain ~20Hz update rate like Arduino
    }	
  
    return 0;
}