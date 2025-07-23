#ifndef MPU6050_ROTATION_H
#define MPU6050_ROTATION_H

#include <stdint.h>

// MPU6050 data structure
typedef struct {
    float AccX, AccY, AccZ;
    float GyroX, GyroY, GyroZ;
    float accAngleX, accAngleY;
    float gyroAngleX, gyroAngleY, gyroAngleZ;
    float roll, pitch, yaw;
    float AccErrorX, AccErrorY;
    float GyroErrorX, GyroErrorY, GyroErrorZ;
} MPU6050_Data;

// Function prototypes
void i2c_init(void);
void mpu6050_init(void);
void mpu6050_calculate_error(void);
void mpu6050_update(void);

// Rotation detection functions
void start_90_degree_rotation(char direction);
uint8_t check_rotation_complete(void);
void print_rotation_status(void);
uint8_t get_rotation_progress(void);
void reset_rotation_detection(void);
void rotation_detection_example(void);

// Utility functions
float get_time_us(void);

// Global variables (extern declarations)
extern MPU6050_Data mpu_data;
extern float currentAngle;
extern float targetAngle;
extern uint8_t rotation_complete;

#endif // MPU6050_ROTATION_H
