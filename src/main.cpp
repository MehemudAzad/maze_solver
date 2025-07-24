#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "serial.h"


//! pin assignments
//! motor pins
#define MOTOR1_IN1    PA0    // Motor 1 direction (Right side, L298N IN1)
#define MOTOR1_IN2    PA1    // Motor 1 direction (L298N IN2)
#define MOTOR2_IN1    PA2    // Motor 2 direction (Left side, L298N IN3)
#define MOTOR2_IN2    PA3    // Motor 2 direction (L298N IN4)
#define EN_MOTOR1     PB3    // Enable motor 1 (OC0 - Timer0, L298N ENA, Right motor)
#define EN_MOTOR2     PD7    // Enable motor 2 (OC2 - Timer2, L298N ENB, Left motor)

//! gyro pins
#define MPU6050_ADDR 0x68
#define ACCEL_SCALE 16384.0 // ±2g range, 16384 LSB/g

// Motor control constants (from Arduino code)
#define MAX_SPEED 255
#define MIN_SPEED 160
#define EQUILIBRIUM_SPEED 248

// Global variables for motor control
uint8_t motor_speed = 200;
uint8_t leftSpeedVal = EQUILIBRIUM_SPEED;
uint8_t rightSpeedVal = MAX_SPEED;
uint8_t isDriving = 0;
uint8_t prevIsDriving = 1;
uint8_t paused = 0;
float targetAngle = 0;
float angle = 0;  // This will be roll * 0.7

// Global gyroscope variable for control functions
float GyroX = 0;

// Helper function for absolute value (integer)
int abs(int x) {
    return (x < 0) ? -x : x;
}




//! motor code
// PWM initialization for motor speed control
void pwm_init() {
    // Set PWM pins as outputs
    DDRB |= (1 << EN_MOTOR1);  // PB3 (OC0) - Right motor
    DDRD |= (1 << EN_MOTOR2);  // PD7 (OC2) - Left motor
    
    // Initialize Timer0 for PWM on PB3 (OC0) - Right motor
    TCCR0 = (1 << WGM01) | (1 << WGM00) | (1 << COM01) | (1 << CS01); // Fast PWM, non-inverting, prescaler 8
    
    // Initialize Timer2 for PWM on PD7 (OC2) - Left motor  
    TCCR2 = (1 << WGM21) | (1 << WGM20) | (1 << COM21) | (1 << CS21); // Fast PWM, non-inverting, prescaler 8
    
    // Start with motors stopped
    OCR0 = 0;  // Right motor speed
    OCR2 = 0;  // Left motor speed
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

// Arduino-style motor functions
int changeSpeed(int motorSpeed, int increment) {
    motorSpeed += increment;
    if (motorSpeed > MAX_SPEED) {
        motorSpeed = MAX_SPEED;
    } else if (motorSpeed < MIN_SPEED) {
        motorSpeed = MIN_SPEED;
    }
    return motorSpeed;
}

// Motor direction control (Arduino-style functions)
void forward() {
    // Set motors to move forward
    PORTA |= (1 << MOTOR1_IN1);   // Right motor forward  
    PORTA &= ~(1 << MOTOR1_IN2);
    PORTA |= (1 << MOTOR2_IN1);   // Left motor forward
    PORTA &= ~(1 << MOTOR2_IN2);
}

void stopCar() {
    // Stop all motors
    PORTA &= ~((1 << MOTOR1_IN1) | (1 << MOTOR1_IN2) | (1 << MOTOR2_IN1) | (1 << MOTOR2_IN2));
    OCR0 = 0;  // Right motor speed = 0
    OCR2 = 0;  // Left motor speed = 0
}

void left() {
    // Turn left - right motor forward, left motor backward
    PORTA &= ~(1 << MOTOR1_IN1);  // Right motor backward
    PORTA |= (1 << MOTOR1_IN2);
    PORTA |= (1 << MOTOR2_IN1);   // Left motor forward  
    PORTA &= ~(1 << MOTOR2_IN2);
}

void right() {
    // Turn right - left motor backward, right motor forward
    PORTA |= (1 << MOTOR1_IN1);   // Right motor forward
    PORTA &= ~(1 << MOTOR1_IN2);
    PORTA &= ~(1 << MOTOR2_IN1);  // Left motor backward
    PORTA |= (1 << MOTOR2_IN2);
}

// Arduino-style driving control function
void driving() {
    int deltaAngle = (int)(targetAngle - angle); // Round equivalent
    forward();
    if (deltaAngle != 0) {
        controlSpeed();
        rightSpeedVal = MAX_SPEED;
        set_right_motor_speed(rightSpeedVal);
        set_left_motor_speed(leftSpeedVal);
    }
}

// Arduino-style speed control function  
void controlSpeed() {
    int deltaAngle = (int)(targetAngle - angle);
    int targetGyroX;
    
    // Proportional control setup
    if (deltaAngle > 30) {
        targetGyroX = 60;
    } else if (deltaAngle < -30) {
        targetGyroX = -60;
    } else {
        targetGyroX = 2 * deltaAngle;
    }
    
    int gyroError = targetGyroX - (int)GyroX;
    
    if (gyroError == 0) {
        // No adjustment needed
    } else if (targetGyroX > GyroX) {
        leftSpeedVal = changeSpeed(leftSpeedVal, -1); // Would increase GyroX
    } else {
        leftSpeedVal = changeSpeed(leftSpeedVal, +1);
    }
}

// Arduino-style rotation function
void rotate() {
    int deltaAngle = (int)(targetAngle - angle);
    int targetGyroX;
    
    if (abs(deltaAngle) <= 1) {
        stopCar();
    } else {
        if (angle > targetAngle) { // Turn left
            left();
        } else if (angle < targetAngle) { // Turn right
            right();
        }
        
        // Proportional control for rotation
        if (abs(deltaAngle) > 30) {
            targetGyroX = 60;
        } else {
            targetGyroX = 2 * abs(deltaAngle);
        }
        
        int gyroError = targetGyroX - (int)fabs(GyroX);
        
        if (gyroError == 0) {
            // No adjustment needed
        } else if (targetGyroX > fabs(GyroX)) {
            leftSpeedVal = changeSpeed(leftSpeedVal, +1); // Would increase abs(GyroX)
        } else {
            leftSpeedVal = changeSpeed(leftSpeedVal, -1);
        }
        
        rightSpeedVal = leftSpeedVal;
        set_right_motor_speed(rightSpeedVal);
        set_left_motor_speed(leftSpeedVal);
    }
}



//! gyro code
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

int main() {
    serial_init(9600);
    
    // Initialize motor control
    motor_init();
    
    // Initialize LED pin (PB0) as output
    DDRB |= (1 << PB0);  // Set PB0 as output for LED
    DDRB |= (1 << PB1);  // Set PB1 as output for LED
    PORTB &= ~(1 << PB0); // Initially turn off LED
    PORTB &= ~(1 << PB1); // Initially turn off LED
    
    TWI_Init();
    MPU6050_Init();

    // Raw sensor data
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    
    // Converted accelerometer and gyroscope values
    float AccX, AccY, AccZ;
    float GyroY, GyroZ; // GyroX is now global
    
    // Angle calculations
    float accAngleX, accAngleY;
    float gyroAngleX = 0, gyroAngleY = 0, yaw = 0;
    float roll = 0, pitch = 0;
    
    // Error compensation values
    float AccErrorX = 0, AccErrorY = 0;
    float GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
    
    // Timing variables (simulating Arduino's micros())
    uint32_t currentTime_us = 0, previousTime_us = 0;
    float elapsedTime_sec;
    
    // Control variables for timing (Arduino-style static variables)
    uint8_t loop_count = 0;
    uint8_t straight_count = 0;
    
    char cmd;
    uint8_t calibrated = 0;

    serial_string("MPU6050 Car Control with Precise 90° Turning Ready\n");
    serial_string("Commands: w=forward, a=left90, d=right90, q=stop, i=info, p=pause, c=calibrate\n");

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
            
            // Timing calculation (simulating Arduino's micros())
            previousTime_us = currentTime_us;
            currentTime_us += 2800; // Approximate loop time in microseconds (~2.8ms)
            elapsedTime_sec = (currentTime_us - previousTime_us) / 1000000.0;
            
            // Calculate angles from accelerometer (gravity vector)
            accAngleX = (atan(AccY / sqrt(AccX*AccX + AccZ*AccZ)) * 180.0 / M_PI) - AccErrorX;
            accAngleY = (atan(-AccX / sqrt(AccY*AccY + AccZ*AccZ)) * 180.0 / M_PI) - AccErrorY;
            
            // Integrate gyroscope data
            gyroAngleX += GyroX * elapsedTime_sec;
            gyroAngleY += GyroY * elapsedTime_sec;
            yaw += GyroZ * elapsedTime_sec;
            
            // Complementary filter (96% gyro + 4% accelerometer)
            roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
            pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
            
            // Apply the 0.7 scaling factor that works for your setup
            angle = roll * 0.7; // This gives accurate 90-degree measurements
        }
        
        // Check for serial commands (Arduino-style command handling)
        if (serial_available()) {
            cmd = serial_read();
            
            if (cmd == 'w' || cmd == 'W') { // Drive forward
                serial_string("forward\n");
                isDriving = 1;
                
            } else if (cmd == 'a' || cmd == 'A') { // Turn left 90°
                serial_string("left\n");
                targetAngle += 90;
                if (targetAngle > 180) {
                    targetAngle -= 360;
                }
                isDriving = 0;
                
            } else if (cmd == 'd' || cmd == 'D') { // Turn right 90°
                serial_string("right\n");
                targetAngle -= 90;
                if (targetAngle <= -180) {
                    targetAngle += 360;
                }
                isDriving = 0;
                
            } else if (cmd == 'q' || cmd == 'Q') { // Stop
                serial_string("stop\n");
                isDriving = 0;
                stopCar();
                
            } else if (cmd == 'i' || cmd == 'I') { // Print info
                if (calibrated) {
                    serial_string("angle: ");
                    serial_num((int16_t)angle);
                    serial_string("° | targetAngle: ");
                    serial_num((int16_t)targetAngle);
                    serial_string("° | GyroX: ");
                    serial_num((int16_t)GyroX);
                    serial_string("° | leftSpeedVal: ");
                    serial_num(leftSpeedVal);
                    serial_string("\n");
                } else {
                    serial_string("Please calibrate first (press 'c')\n");
                }
                
            } else if (cmd == 'p' || cmd == 'P') { // Pause
                paused = !paused;
                stopCar();
                isDriving = 0;
                serial_string("Program paused/unpaused\n");
                
            } else if (cmd == 'c' || cmd == 'C') { // Calibrate
                serial_string("Calibrating... Keep sensor stationary!\n");
                stopCar();
                
                // Reset error values
                AccErrorX = 0; AccErrorY = 0;
                GyroErrorX = 0; GyroErrorY = 0; GyroErrorZ = 0;
                
                // Calibration with 200 samples (Arduino-style)
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
                targetAngle = 0;
                currentTime_us = 0; previousTime_us = 0;
                
                calibrated = 1;
                serial_string("Calibration complete!\n");
            }
        }
        
        // Main control logic (Arduino-style with loop counter)
        if (loop_count < 6) {
            loop_count++;
        } else {
            loop_count = 0; // Reset counter - runs every 7 loops (~19.6ms at 2.8ms per loop)
            
            if (!paused && calibrated) {
                // Detect mode change
                if (isDriving != prevIsDriving) {
                    leftSpeedVal = EQUILIBRIUM_SPEED;
                    straight_count = 0;
                    serial_string("mode changed, isDriving: ");
                    serial_num(isDriving);
                    serial_string("\n");
                }
                
                if (isDriving) {
                    // Check if driving straight for equilibrium speed calculation
                    if (fabs(targetAngle - angle) < 3) {
                        if (straight_count < 20) {
                            straight_count++;
                        } else {
                            straight_count = 0;
                            // Update equilibrium speed based on current performance
                            serial_string("EQUILIBRIUM reached, leftSpeedVal: ");
                            serial_num(leftSpeedVal);
                            serial_string("\n");
                        }
                    } else {
                        straight_count = 0;
                    }
                    driving(); // Arduino-style driving function
                } else {
                    rotate(); // Arduino-style rotation function
                }
                
                prevIsDriving = isDriving;
            }
        }
        
        _delay_ms(3); // Approximate Arduino loop timing (~2.8ms)
    }
    
    return 0;
}