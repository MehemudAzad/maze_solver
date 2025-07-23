#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "lcd.h"  // I2C LCD functions
#include "twi.h"  // TWI/I2C functions

#define MPU6050_ADDR 0x68
#define RAD_TO_DEG 57.2958

// ===== TWI Read Helpers =====
unsigned char TWI_Read_ACK() {
  TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWEA);
  while (!(TWCR & (1<<TWINT)));
  return TWDR;
}

unsigned char TWI_Read_NACK() {
  TWCR = (1<<TWEN)|(1<<TWINT);
  while (!(TWCR & (1<<TWINT)));
  return TWDR;
}

// ===== MPU6050 Init =====
void MPU6050_Init() {
  TWI_Start();
  TWI_Write((MPU6050_ADDR << 1)); // write mode
  TWI_Write(0x6B);  // Power management register
  TWI_Write(0x00);  // Wake up MPU6050
  TWI_Stop();
}

// ===== Read Accelerometer Data =====
int16_t MPU6050_ReadAccel(char axis) {
  uint8_t reg;
  switch (axis) {
    case 'X': reg = 0x3B; break;
    case 'Y': reg = 0x3D; break;
    case 'Z': reg = 0x3F; break;
    default: return 0;
  }

  TWI_Start();
  TWI_Write((MPU6050_ADDR << 1));
  TWI_Write(reg);
  TWI_Stop();

  TWI_Start();
  TWI_Write((MPU6050_ADDR << 1) | 1);
  uint8_t high = TWI_Read_ACK();
  uint8_t low  = TWI_Read_NACK();
  TWI_Stop();

  return (int16_t)((high << 8) | low);
}

// ===== MAIN FUNCTION =====
int main(void) {
  char buffer[16];
  int16_t ax, ay, az;
  float pitch;

  // Init
  TWI_Init();
  LCD_Init();
  MPU6050_Init();

  DDRC |= (1 << PC0);  // Set PC0 as output for LED

  while (1) {
    // Read accelerometer
    ax = MPU6050_ReadAccel('X');
    ay = MPU6050_ReadAccel('Y');
    az = MPU6050_ReadAccel('Z');

    // Compute pitch angle
    pitch = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    // Display pitch on LCD
    LCD_SetCursor(0, 0);
    LCD_SendString("Pitch: ");
    dtostrf(pitch, 6, 2, buffer);
    LCD_SendString(buffer);
    LCD_SendString("   "); // Clear leftover chars

    // Check for tilt and trigger LED
    if (pitch > 45.0 || pitch < -45.0) {
      PORTC |= (1 << PC0);  // Turn ON LED
    } else {
      PORTC &= ~(1 << PC0); // Turn OFF LED
    }

    _delay_ms(500);
  }
}