#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "serial.h"

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

	serial_string("MPU6050 Advanced Angle Sensor Ready\n");
	serial_string("Press 'C' to calibrate (keep stationary!)\n");
	serial_string("Press 'G' to get angles and sensor data\n");

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
		}
		
		// Check for bluetooth commands
		if (serial_available()) {
			cmd = serial_read();
			
			if (cmd == 'C' || cmd == 'c') {
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
				roll = 0; pitch = 0;
				currentTime_ms = 0;
				previousTime_ms = 0;
				
				calibrated = 1;
				serial_string("Calibration complete!\n");
				
			} else if (cmd == 'G' || cmd == 'g') {
				if (calibrated) {
					serial_string("Roll: ");
					serial_num((int16_t)roll);
					serial_string("° | Pitch: ");
					serial_num((int16_t)pitch);
					serial_string("° | Yaw: ");
					serial_num((int16_t)yaw);
					serial_string("° | X: ");
					serial_num(accelX);
					serial_string(" | Y: ");
					serial_num(accelY);
					serial_string(" | Z: ");
					serial_num(accelZ);
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