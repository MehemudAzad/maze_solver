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
