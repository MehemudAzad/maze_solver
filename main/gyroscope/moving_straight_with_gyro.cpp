#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "serial.h"


#define MOTOR1_IN1    PA0    // Motor 1 direction (Right side, L298N IN1)
#define MOTOR1_IN2    PA1    // Motor 1 direction (L298N IN2)
#define MOTOR2_IN1    PA2    // Motor 2 direction (Left side, L298N IN3)
#define MOTOR2_IN2    PA3    // Motor 2 direction (L298N IN4)
#define EN_MOTOR1     PB3    // Enable motor 1 (OC0 - Timer0, L298N ENA, Right motor)
#define EN_MOTOR2     PD7    // Enable motor 2 (OC2 - Timer2, L298N ENB, Left motor)
// #define SONAR_TRIG PD2
// #define SONAR_ECHO PD3

// #define SONAR2_TRIG PD4
// #define SONAR2_ECHO PD5

#define IR_SENSOR_PIN PC0 // Example pin for IR sensor
void motor_stop(void);
void motor_forward(void);
void motor_reverse(void);
void motor_left(void);
void motor_right(void);
void set_right_motor_speed(uint8_t speed);
void set_left_motor_speed(uint8_t speed);

// Global variables for straight-line control only
float currentAngle = 0;     // Current roll angle * 0.7
float forwardSetpointAngle = 0; // Target roll angle for straight movement
const float Kp = 2.5;         // Proportional gain for correction (tune if needed)
const int RIGHT_MOTOR_OFFSET = 4; // Speed difference for right motor

// Speed control variables
uint8_t motor_speed = 120;  // Default speed (0-255)


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

// Motor direction control - continuous straight movement with gyro correction
void motor_forward_straight() {
    // Set motor directions for forward movement
    PORTA &= ~((1 << MOTOR1_IN1) | (1 << MOTOR2_IN1));
    PORTA |= (1 << MOTOR1_IN2) | (1 << MOTOR2_IN2);
    
    // Proportional control to maintain a straight line using gyro
    float error = currentAngle - forwardSetpointAngle;
    int16_t correction = (int16_t)(Kp * error);

    // Calculate new motor speeds with offset for the right motor
    int16_t rightSpeed = motor_speed + RIGHT_MOTOR_OFFSET + correction;
    int16_t leftSpeed = motor_speed - correction;

    // Clamp speeds to valid PWM range (0-255)
    if (rightSpeed > 255) rightSpeed = 255;
    if (rightSpeed < 0) rightSpeed = 0;
    if (leftSpeed > 255) leftSpeed = 255;
    if (leftSpeed < 0) leftSpeed = 0;

    set_right_motor_speed((uint8_t)rightSpeed);
    set_left_motor_speed((uint8_t)leftSpeed);
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
    serial_string("inside motor right");
    PORTA &= ~((1 << MOTOR1_IN1) | (1 << MOTOR2_IN2));
    PORTA |= (1 << MOTOR1_IN2) | (1 << MOTOR2_IN1);
    set_right_motor_speed(motor_speed); // Slower right motor
    set_left_motor_speed(motor_speed);      // Full speed left motor
}

void motor_left() {
    serial_string("inside motor left");
	PORTA &= ~((1 << MOTOR1_IN2) | (1 << MOTOR2_IN1));
    PORTA |= (1 << MOTOR1_IN1) | (1 << MOTOR2_IN2);
    set_right_motor_speed(motor_speed);     // Full speed right motor
    set_left_motor_speed(motor_speed);  // Slower left motor
}




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
  motor_init();
  motor_stop();
	
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
	
	// Error compensation values
	float AccErrorX = 0, AccErrorY = 0;
	float GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
	
	// Timing for gyroscope integration
	uint32_t currentTime_ms, previousTime_ms, elapsedTime_ms;
	float elapsedTime_sec;
	
	char cmd;
	uint8_t calibrated = 0;
	uint8_t straightTestActive = 0;

	serial_string("Gyro Straight-Line Test Ready\n");
	serial_string("Press 'C' to calibrate (keep stationary!)\n");
	serial_string("Press 'S' to start straight movement test\n");
	serial_string("Press 'T' to stop the test\n");

	// Initialize timing with a simple millisecond counter
	currentTime_ms = 0;
	previousTime_ms = 0;

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
			
			// Better timing calculation
			previousTime_ms = currentTime_ms;
			currentTime_ms += 50; // 50ms loop time
			elapsedTime_ms = currentTime_ms - previousTime_ms;
			elapsedTime_sec = elapsedTime_ms / 1000.0; // Convert to seconds
			
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
			
			// Update current angle with your working scaling factor
			currentAngle = roll * 0.7;
		}
		
		// Continuous straight movement test
		if (calibrated && straightTestActive) {
			motor_forward_straight();
		}
		
		// Check for bluetooth commands
		if (serial_available()) {
			cmd = serial_read();
			
			if (cmd == 'C' || cmd == 'c') {
				serial_string("Calibrating... Keep sensor stationary!\n");
				motor_stop();
				straightTestActive = 0;
				
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
				roll = 0; pitch = 0;
				currentTime_ms = 0;
				previousTime_ms = 0;
				
				calibrated = 1;
				serial_string("Calibration complete!\n");
				serial_string("Ready for straight-line test. Press 'S' to start.\n");
				
			} else if (cmd == 'S' || cmd == 's') {
				if (calibrated) {
					forwardSetpointAngle = currentAngle; // Set current angle as target
					straightTestActive = 1;
					serial_string("Starting continuous straight movement test...\n");
					serial_string("Target angle: ");
					serial_num((int16_t)forwardSetpointAngle);
					serial_string("°\n");
				} else {
					serial_string("Please calibrate first (press 'C')\n");
				}
			} else if (cmd == 'T' || cmd == 't') {
				motor_stop();
				straightTestActive = 0;
				serial_string("Straight movement test STOPPED.\n");
			} else if (cmd == 'G' || cmd == 'g') {
				if (calibrated) {
					serial_string("Current angle: ");
					serial_num((int16_t)currentAngle);
					serial_string("° | Target: ");
					serial_num((int16_t)forwardSetpointAngle);
					serial_string("° | Error: ");
					serial_num((int16_t)(currentAngle - forwardSetpointAngle));
					serial_string("° | Test: ");
					serial_num(straightTestActive);
					serial_string("\n");
				} else {
					serial_string("Please calibrate first (press 'C')\n");
				}
			}
		}
    
		_delay_ms(5); // 20Hz update rate
	}	
  
  return 0;
}