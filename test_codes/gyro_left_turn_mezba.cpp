#define F_CPU 5000000UL // 5 MHz
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

// Motor control pins
#define IN1 PA0
#define IN2 PA1
#define IN3 PA6
#define IN4 PA7

// Sensor pins
#define TRIG_F PD0 // Front sensor
#define ECHO_F PD1
#define TRIG_L PD2 // Left sensor
#define ECHO_L PD3
#define TRIG_R PD4 // Right sensor
#define ECHO_R PD5

// Gyroscope (MPU-6050) I2C settings
#define MPU6050_ADDR 0x68     // I2C address of MPU-6050 (AD0 = GND)
#define I2C_SCL_FREQ 100000UL // 100 kHz I2C clock
#define TIMEOUT 30000         // Timeout for ultrasonic sensors

// MPU-6050 registers
#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B
#define GYRO_ZOUT_H 0x47
#define OBSTACLE_DISTANCE 20 // Distance threshold in cm

// Function prototypes
void init_ports();
void init_timer1();
void init_twi();
void mpu6050_init();
int16_t read_gyro_z();
uint16_t get_distance(uint8_t trig_pin, uint8_t echo_pin);
void move_forward();
void turn_left();
void turn_right();
void stop();

// TWI (I2C) functions
void twi_start()
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Send START
    while (!(TWCR & (1 << TWINT)))
        ; // Wait for TWINT
}

void twi_stop()
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN); // Send STOP
}

void twi_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
    while (!(TWCR & (1 << TWINT)))
        ; // Wait for completion
}

uint8_t twi_read_ack()
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // Read with ACK
    while (!(TWCR & (1 << TWINT)))
        ;
    return TWDR;
}

uint8_t twi_read_nack()
{
    TWCR = (1 << TWINT) | (1 << TWEN); // Read with NACK
    while (!(TWCR & (1 << TWINT)))
        ;
    return TWDR;
}

void init_ports()
{
    // Motor pins
    DDRA |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4); // Set as outputs
    PORTA = 0x00;                                              // All motors off initially

    // Sensor pins
    DDRD |= (1 << TRIG_F) | (1 << TRIG_L) | (1 << TRIG_R);    // TRIG pins as outputs
    DDRD &= ~((1 << ECHO_F) | (1 << ECHO_L) | (1 << ECHO_R)); // ECHO pins as inputs

    // I2C pins (PC0: SCL, PC1: SDA) are configured by TWI module
}

void init_timer1()
{
    TCCR1B = 0;            // Stop timer
    TCNT1 = 0;             // Clear timer
    TCCR1B |= (1 << CS11); // Prescaler 8 (5 MHz / 8 = 625 kHz, 1.6µs per tick)
}

void init_twi()
{
    // Set I2C clock: TWBR = ((F_CPU / I2C_SCL_FREQ) - 16) / (2 * prescaler)
    // Prescaler = 1, TWBR = ((5000000 / 100000) - 16) / 2 = 17
    TWBR = 17;
    TWSR = 0;           // Prescaler = 1
    TWCR = (1 << TWEN); // Enable TWI
}

void mpu6050_init()
{
    // Wake up MPU-6050 (disable sleep mode)
    twi_start();
    twi_write(MPU6050_ADDR << 1); // Write mode
    twi_write(PWR_MGMT_1);
    twi_write(0x00); // Clear sleep bit
    twi_stop();

    // Set gyro full-scale range (±250°/s)
    twi_start();
    twi_write(MPU6050_ADDR << 1);
    twi_write(GYRO_CONFIG);
    twi_write(0x00); // ±250°/s
    twi_stop();
}

int16_t read_gyro_z()
{
    // Read Z-axis gyro data (2 bytes: GYRO_ZOUT_H and GYRO_ZOUT_L)
    twi_start();
    twi_write(MPU6050_ADDR << 1);
    twi_write(GYRO_ZOUT_H);
    twi_stop();

    twi_start();
    twi_write((MPU6050_ADDR << 1) | 1); // Read mode
    uint8_t high = twi_read_ack();
    uint8_t low = twi_read_nack();
    twi_stop();

    return (int16_t)((high << 8) | low); // Combine high and low bytes
}

uint16_t get_distance(uint8_t trig_pin, uint8_t echo_pin)
{
    PORTD |= (1 << trig_pin);
    _delay_us(10);
    PORTD &= ~(1 << trig_pin);

    TCNT1 = 0;
    uint16_t timeout = TIMEOUT;
    while (!(PIND & (1 << echo_pin)) && timeout--)
        _delay_us(1);
    if (timeout == 0)
        return 999;

    TCNT1 = 0;
    TCCR1B |= (1 << CS11);
    while ((PIND & (1 << echo_pin)) && TCNT1 < TIMEOUT)
        ;
    TCCR1B = 0;
    uint16_t ticks = TCNT1;
    return ticks / 73; // Distance in cm (5 MHz, prescaler 8)
}

void move_forward()
{
    PORTA = (1 << IN1) | (0 << IN2) | (1 << IN3) | (0 << IN4);
    _delay_ms(5);
}

void turn_left()
{
    // Target ~90-degree turn using gyro
    int16_t initial_z = read_gyro_z();
    int32_t angle_sum = 0;
    PORTA = (1 << IN1) | (0 << IN2) | (0 << IN3) | (1 << IN4); // Left forward, right backward
    while (abs(angle_sum) < 9000)
    { // Approx 90° (adjust based on calibration)
        int16_t gyro_z = read_gyro_z();
        angle_sum += gyro_z / 131; // ±250°/s, 131 LSB/(°/s)
        _delay_ms(10);
    }
    stop();
}

void turn_right()
{
    // Target ~90-degree turn using gyro
    int16_t initial_z = read_gyro_z();
    int32_t angle_sum = 0;
    PORTA = (0 << IN1) | (1 << IN2) | (1 << IN3) | (0 << IN4); // Left backward, right forward
    while (abs(angle_sum) < 9000)
    { // Approx 90° (adjust based on calibration)
        int16_t gyro_z = read_gyro_z();
        angle_sum += gyro_z / 131; // ±250°/s, 131 LSB/(°/s)
        _delay_ms(10);
    }
    stop();
}

void stop()
{
    PORTA = 0x00;
    _delay_ms(10);
}

int main(void)
{
    init_ports();
    init_timer1();
    init_twi();
    mpu6050_init();

    while (1)
    {
        uint16_t front = get_distance(TRIG_F, ECHO_F);
        _delay_ms(10);
        uint16_t left = get_distance(TRIG_L, ECHO_L);
        _delay_ms(10);
        uint16_t right = get_distance(TRIG_R, ECHO_R);
        _delay_ms(10);

        // Obstacle avoidance with gyro-assisted turns
        if (front > OBSTACLE_DISTANCE && front < 999)
        {
            move_forward();
        }
        else if (left > right && left < 999)
        {
            turn_left();
        }
        else if (right < 999)
        {
            turn_right();
        }
        else
        {
            stop();
        }

        _delay_ms(100);
    }
    return 0;
}