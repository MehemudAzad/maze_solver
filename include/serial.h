/****************************************************************************
 Title:    Serial library
 Company:  www.digitlabz.com
 File:     serial.h
 Target:   any AVR device, AVR-GCC
*****************************************************************************/

#ifndef SERIAL_H
#define SERIAL_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdint.h>

#define RECEIVE_BUFF_SIZE 300
#define USART_RXC_VECT USART_RXC_vect

// ——— Public API ——————————————————————————————————————————————
void serial_init(uint16_t baudrate);
void serial_finit(uint16_t baudrate, uint8_t xtal);
uint8_t serial_available(void);
unsigned char serial_read(void);
void serial_char(char data);
void serial_string(const char *str);
void serial_string_P(const char *progmem_s);
void serial_num(int num);
void serial_numl(long num);
void s_readbuffer(void *buff, uint16_t len);
char serial_nreadw(char *msg, uint8_t length);
char serial_nread(char *msg, uint8_t length);
void s_clearbuffer(void);
void urclear(void);

// ——— Internal ring-buffer state —————————————————————————————————
volatile char URBuff[RECEIVE_BUFF_SIZE];
volatile int16_t UQFront;
volatile int16_t UQEnd;

// optional debug storage
volatile char char1;

// ——— Implementations ————————————————————————————————————————

static inline void _serial_enqueue(char data)
{
    // advance end pointer, drop oldest if full
    if (((UQEnd == RECEIVE_BUFF_SIZE - 1) && UQFront == 0) ||
        ((UQEnd + 1) == UQFront))
    {
        UQFront++;
        if (UQFront == RECEIVE_BUFF_SIZE)
            UQFront = 0;
    }

    if (UQEnd == RECEIVE_BUFF_SIZE - 1)
        UQEnd = 0;
    else
        UQEnd++;

    URBuff[UQEnd] = data;
    if (UQFront == -1)
        UQFront = 0;
}

void serial_init(uint16_t baudrate)
{
    // reset buffer pointers
    UQFront = UQEnd = -1;

    // enable double-speed mode (8× oversampling)
    UCSRA = (1 << U2X);

    // calculate UBRR for double-speed: UBRR = (F_CPU / (8 * baudrate)) - 1
    uint16_t ubrr = (F_CPU / (8UL * baudrate)) - 1;
    UBRRH = (uint8_t)(ubrr >> 8);
    UBRRL = (uint8_t)ubrr;

    // 8N1, enable RX interrupt, enable TX/RX
    UCSRC = (1 << URSEL) | (3 << UCSZ0);
    UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);

    sei();
}

void serial_finit(uint16_t baudrate, uint8_t xtal)
{
    uint32_t clk = (uint32_t)xtal * 1000000UL;
    uint16_t ubrrvalue = ((clk + (baudrate * 8UL)) / (baudrate * 16UL) - 1);

    UQFront = UQEnd = -1;
    UBRRH = (uint8_t)(ubrrvalue >> 8);
    UBRRL = (uint8_t)ubrrvalue;
    UCSRC = (1 << URSEL) | (3 << UCSZ0);
    UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
    sei();
}

unsigned char serial_read()
{
    char data;
    if (UQFront == -1)
        return 0;
    data = URBuff[UQFront];
    if (UQFront == UQEnd)
    {
        UQFront = UQEnd = -1;
    }
    else
    {
        UQFront++;
        if (UQFront == RECEIVE_BUFF_SIZE)
            UQFront = 0;
    }
    return data;
}

uint8_t serial_available()
{
    if (UQFront == -1)
        return 0;
    if (UQFront <= UQEnd)
        return (UQEnd - UQFront + 1);
    else
        return (RECEIVE_BUFF_SIZE - UQFront + UQEnd + 1);
}

void serial_char(char data)
{
    while (!(UCSRA & (1 << UDRE)))
        ;
    UDR = data;
}

void serial_string(const char *str)
{
    while (*str)
    {
        serial_char(*str++);
    }
}

void serial_string_P(const char *progmem_s)
{
    char c;
    while ((c = pgm_read_byte(progmem_s++)) != '\0')
    {
        serial_char(c);
    }
}

void serial_numl(long num)
{
    char buffer[12];
    ltoa(num, buffer, 10);
    serial_string(buffer);
}

void serial_num(int num)
{
    char buffer[7];
    itoa(num, buffer, 10);
    serial_string(buffer);
}

void s_readbuffer(void *buff, uint16_t len)
{
    uint8_t *b = (uint8_t *)buff; // treat buff as byte-pointer
    for (uint16_t i = 0; i < len; i++)
    {
        b[i] = serial_read();
    }
}

char serial_nreadw(char *msg, uint8_t length)
{
    while (serial_available() < length)
        ;
    s_readbuffer(msg, length);
    return 1;
}

char serial_nread(char *msg, uint8_t length)
{
    if (serial_available() >= length)
    {
        s_readbuffer(msg, length);
        return 1;
    }
    else
    {
        msg[0] = '\0';
        return 0;
    }
}

void s_clearbuffer()
{
    while (serial_available() > 0)
        serial_read();
}

void urclear()
{
    for (uint16_t i = 0; i < RECEIVE_BUFF_SIZE; i++)
        URBuff[i] = '\0';
}

// USART RX interrupt handler
ISR(USART_RXC_VECT)
{
    char data = UDR;
    char1 = data; // optional debug storage
    _serial_enqueue(data);
}

#endif // SERIAL_H



// this should be integnrated with the main code of sonar 