#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "lcd.h" // I2C LCD functions
#include "twi.h" // TWI/I2C functions

#define MPU6050_ADDR 0x68
#define RAD_TO_DEG 57.2958

// ===== TWI Read Helpers =====
unsigned char TWI_Read_ACK()
{
    TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)))
        ;
    return TWDR;
}

unsigned char TWI_Read_NACK()
{
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)))
        ;
    return TWDR;
}

// ===== MPU6050 Init =====
void MPU6050_Init()
{
    TWI_Start();
    TWI_Write((MPU6050_ADDR << 1)); // write mode
    TWI_Write(0x6B);                // Power management register
    TWI_Write(0x00);                // Wake up MPU6050
    TWI_Stop();
}

// ===== Read Accelerometer Data =====
int16_t MPU6050_ReadAccel(char axis)
{
    uint8_t reg;
    switch (axis)
    {
    case 'X':
        reg = 0x3B;
        break;
    case 'Y':
        reg = 0x3D;
        break;
    case 'Z':
        reg = 0x3F;
        break;
    default:
        return 0;
    }

    TWI_Start();
    TWI_Write((MPU6050_ADDR << 1));
    TWI_Write(reg);
    TWI_Stop();

    TWI_Start();
    TWI_Write((MPU6050_ADDR << 1) | 1);
    uint8_t high = TWI_Read_ACK();
    uint8_t low = TWI_Read_NACK();
    TWI_Stop();

    return (int16_t)((high << 8) | low);
}

// ===== MAIN FUNCTION =====
int main(void)
{
    char buffer[16];
    int16_t ax, ay, az;

    float pitch;

    // Init
    TWI_Init();
    LCD_Init();
    MPU6050_Init();

    DDRB |= (1 << PB0); // Set PB0 as output for LED

    while (1)
    {
        // Read accelerometer
        ax = MPU6050_ReadAccel('X');
        ay = MPU6050_ReadAccel('Y');
        az = MPU6050_ReadAccel('Z');

        // Convert to float first to avoid overflow
        float fax = (float)ax;
        float fay = (float)ay;
        float faz = (float)az;

        // Compute pitch angle
        // pitch = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
        pitch = atan2(fax, sqrt(fay * fay + faz * faz)) * (180.0 / 3.14159265);

        // Display pitch on LCD
        LCD_SetCursor(0, 0);
        LCD_SendString("Pitch: ");
        dtostrf(pitch, 6, 2, buffer);
        LCD_SendString(buffer);
        LCD_SendString("   "); // Clear leftover chars

        // Check for tilt and trigger LED
        LCD_SetCursor(1, 0);
        LCD_SendString("LED: ");
        if (pitch > 45.0 || pitch < -45.0)
        {
            PORTB |= (1 << PB0); // Turn ON LED
            LCD_SendString("ON ");
            _delay_ms(1000);
        }
        else
        {
            PORTB &= ~(1 << PB0); // Turn OFF LED
            LCD_SendString("OFF");
            _delay_ms(1000);
        }

        _delay_ms(500);
    }
}

#include <avr/io.h>
#include <util/delay.h>
#include "twi.h"
#include "lcd.h"

#define LCD_ADDR 0x27 // Change to 0x3F if needed
#define LCD_BACKLIGHT 0x08

void LCD_WriteNibble(unsigned char nibble)
{
    // Send with Enable high
    TWI_Start();
    TWI_Write((LCD_ADDR << 1));
    TWI_Write(nibble | LCD_BACKLIGHT | 0x04); // EN = 1
    TWI_Write(nibble | LCD_BACKLIGHT);        // EN = 0
    TWI_Stop();
    _delay_us(50);
}

void LCD_SendCommand(unsigned char cmd)
{
    LCD_WriteNibble(cmd & 0xF0);         // upper nibble
    LCD_WriteNibble((cmd << 4) & 0xF0);  // lower nibble
}

void LCD_SendChar(unsigned char data)
{
    LCD_WriteNibble((data & 0xF0) | 0x01);         // RS = 1
    LCD_WriteNibble(((data << 4) & 0xF0) | 0x01);  // RS = 1
}

void LCD_SendString(char *str)
{
    while (*str)
        LCD_SendChar(*str++);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    LCD_SendCommand(addr);
}

void LCD_Init(void)
{
    _delay_ms(50);
    LCD_WriteNibble(0x30);
    _delay_ms(5);
    LCD_WriteNibble(0x30);
    _delay_us(100);
    LCD_WriteNibble(0x30);
    LCD_WriteNibble(0x20); // 4-bit mode

    LCD_SendCommand(0x28); // 2 lines, 5x7 matrix
    LCD_SendCommand(0x0C); // Display ON, Cursor OFF
    LCD_SendCommand(0x01); // Clear screen
    _delay_ms(2);
    LCD_SendCommand(0x06); // Entry mode
}

#include <avr/io.h>
#include "twi.h"

void TWI_Init(void)
{
    TWBR = 32;        // 100kHz SCL with F_CPU = 1MHz
    TWSR = 0x00;      // Prescaler = 1
}

void TWI_Start(void)
{
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

void TWI_Stop(void)
{
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
}

void TWI_Write(unsigned char data)
{
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}
