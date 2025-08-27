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


//! angle for turning
#define ANGLE_LEFT 81
#define ANGLE_RIGHT 80
//! sonar code
#define SONAR_TRIG PD2
#define SONAR_ECHO PD3

#define SONAR2_TRIG PD4
#define SONAR2_ECHO PD5

#define SONAR3_TRIG PB1
#define SONAR3_ECHO PB2

//! sonar init
void sonar_init(void) {
    DDRD |= (1 << SONAR_TRIG);  // TRIG as output
    DDRD &= ~(1 << SONAR_ECHO); // ECHO as input
}

void sonar2_init(void) {
    DDRD |= (1 << SONAR2_TRIG);  // TRIG2 as output
    DDRD &= ~(1 << SONAR2_ECHO); // ECHO2 as input
}

void sonar3_init(void) {
    DDRB |= (1 << SONAR3_TRIG);  // TRIG3 as output
    DDRB &= ~(1 << SONAR3_ECHO); // ECHO3 as input
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
    //! calibrated to return values in cm
    return (uint16_t)(count )*2.3/10;
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
    return (uint16_t)(count )*2.3/10;
}

//get distance from front sonar
uint16_t sonar3_get_distance_cm(void) {
    PORTB &= ~(1 << SONAR3_TRIG);
    _delay_us(2);
    PORTB |= (1 << SONAR3_TRIG);
    _delay_us(10);
    PORTB &= ~(1 << SONAR3_TRIG);

    uint32_t timeout = 30000;
    while (!(PINB & (1 << SONAR3_ECHO))) {
        if (--timeout == 0) return 0xFFFF;
    }
    uint32_t count = 0;
    while (PINB & (1 << SONAR3_ECHO)) {
        count++;
        _delay_us(1);
        if (count > 30000) return 0xFFFF;
    }
    return (uint16_t)(count)*2.3/10; // Calibrated to return values in cm
}

//! ir sensor
#define IR_SENSOR_PIN PB0 // Example pin for IR sensor

//! Maze navigation thresholds
#define FRONT_OBSTACLE_THRESHOLD_CM 13 // Distance in cm to trigger left turn
#define FRONT2_OBSTACLE_THRESHOLD_CM 15 // Distance in cm to trigger left turn
#define WALL_DISTANCE_THRESHOLD_CM 25  // Distance threshold for wall detection (left-hand rule)
#define FORWARD_MOVE_DURATION_MS 900   // Duration to move forward after turns

void motor_stop(void);
void motor_forward(void);
void motor_reverse(void);
void motor_left(void);
void motor_right(void);
void set_right_motor_speed(uint8_t speed);
void set_left_motor_speed(uint8_t speed);

// Global variables for turning control
uint8_t isTurning = 0;      // 0=not turning, 1=left turn, 2=right turn
float startAngle = 0;       // Angle when turn started
float currentAngle = 0;     // Current roll angle * 0.7
uint8_t sequenceStep = 0;   // 0=left, 1=right, 2=forward, 3=done
const int RIGHT_MOTOR_OFFSET = 4; // Speed difference for right motor

// Turn counter for auto-recalibration
uint8_t turnCounter = 0;    // Count completed turns
#define RECALIBRATION_INTERVAL 2  // Recalibrate every 2 turns

// Global variables for straight-line control
uint8_t isMovingStraight = 0; // Flag for straight movement mode
float forwardSetpointAngle = 0; // Target roll angle for straight movement
const float Kp = 2.5;         // Proportional gain for gyro correction (tune if needed)
const float Kp_avoid = 2.0;   // Proportional gain for collision avoidance (tune if needed)
uint16_t forward_duration_counter = 0; // Counter for forward movement time
#define FORWARD_DURATION 20   // 20 loops * 50ms/loop = 1000ms
#define COLLISION_AVOIDANCE_THRESHOLD 5  // Activate avoidance when closer than 5cm

// Speed control variables
uint8_t motor_speed = 110;  // Default speed (0-255)
uint8_t speed_increment = 25; // Speed change step
uint8_t control_mode = 0;   // 0 = Bluetooth, 1 = Gesture

uint8_t ir_sensor_read(void) {
    // Configure IR sensor pin as input with pull-up
    DDRB &= ~(1 << IR_SENSOR_PIN);  // Set as input
    PORTB |= (1 << IR_SENSOR_PIN);  // Enable pull-up resistor
 
    // Read the sensor (0 = obstacle detected, 1 = no obstacle)
    return (PINB & (1 << IR_SENSOR_PIN)) ? 1 : 0;

}

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

void motor_init() {
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

// Motor direction control
void motor_forward_straight() {
    // Set initial motor directions for forward movement
    PORTA &= ~((1 << MOTOR1_IN1) | (1 << MOTOR2_IN1));
    PORTA |= (1 << MOTOR1_IN2) | (1 << MOTOR2_IN2);
    // set_right_motor_speed(motor_speed);
	  // set_left_motor_speed(motor_speed+16.5);

    // Record the starting angle for straight-line correction
    forwardSetpointAngle = currentAngle;
    isMovingStraight = 1;
    forward_duration_counter = 0; // Reset duration counter
    serial_string("Moving FORWARD (straight) for 1000ms...\n");
}

void correct_straight_path() {
    if (!isMovingStraight) return;

    // Left-hand rule maze navigation algorithm
    uint16_t left_distance = sonar_get_distance_cm();     // Left sensor
    uint16_t front_distance = sonar3_get_distance_cm();   // Front sensor  
    uint16_t right_distance = sonar2_get_distance_cm();   // Right sensor
    
    // Skip left-hand rule decisions if we're in distance-based movement after turns
    if (sequenceStep == 6 || sequenceStep == 7) {
        // Only do gyro correction, collision avoidance, and obstacle avoidance during distance-based movement
        if (front_distance > FRONT2_OBSTACLE_THRESHOLD_CM && front_distance != 0xFFFF) {
            // Collision avoidance logic: steer away if too close to side obstacles
            float avoidance_correction = 0;
            
            // Check left sensor for collision avoidance
            if (left_distance != 0xFFFF && left_distance < COLLISION_AVOIDANCE_THRESHOLD) {
                // Too close to left obstacle - steer RIGHT (positive correction)
                float left_error = COLLISION_AVOIDANCE_THRESHOLD - left_distance;
                avoidance_correction += Kp_avoid * left_error;
                
                // serial_string("COLLISION AVOIDANCE: Left obstacle at ");
                // serial_num(left_distance);
                // serial_string("cm - steering RIGHT\n");
            }
            
            // Check right sensor for collision avoidance
            if (right_distance != 0xFFFF && right_distance < COLLISION_AVOIDANCE_THRESHOLD) {
                // Too close to right obstacle - steer LEFT (negative correction)
                float right_error = COLLISION_AVOIDANCE_THRESHOLD - right_distance;
                avoidance_correction -= Kp_avoid * right_error;
                
                // serial_string("COLLISION AVOIDANCE: Right obstacle at ");
                // serial_num(right_distance);
                // serial_string("cm - steering LEFT\n");
            }
            
            // Gyroscope correction for maintaining straight line
            float gyro_error = currentAngle - forwardSetpointAngle;
            int16_t gyro_correction = (int16_t)(Kp * gyro_error);
            
            // Combine collision avoidance and gyro corrections
            int16_t total_correction = gyro_correction + (int16_t)avoidance_correction;
            
            // Calculate new motor speeds with offset for the right motor
            int16_t rightSpeed = motor_speed + RIGHT_MOTOR_OFFSET + total_correction;
            int16_t leftSpeed = motor_speed - total_correction;

            // Clamp speeds to valid PWM range (0-255)
            if (rightSpeed > 255) rightSpeed = 255;
            if (rightSpeed < 0) rightSpeed = 0;
            if (leftSpeed > 255) leftSpeed = 255;
            if (leftSpeed < 0) leftSpeed = 0;

            set_right_motor_speed((uint8_t)rightSpeed);
            set_left_motor_speed((uint8_t)leftSpeed);
            return;
        } else {
            // Stop if obstacle detected during distance movement
            motor_stop();
            isMovingStraight = 0;
            // serial_string("Obstacle detected during distance movement - stopping\n");
            sequenceStep = 0; // Reset to normal navigation
            return;
        }
    }
    
    // Normal left-hand rule navigation (only when not in distance-based movement)
    // Priority 1: Turn left if there's enough space on the left (>30cm)
    if (left_distance > WALL_DISTANCE_THRESHOLD_CM && left_distance != 0xFFFF) {
        //! we added this delay to keep the car moving
        _delay_ms(400); // Move forward for a brief moment
        motor_stop();
        isMovingStraight = 0;
        // serial_string("Left space available (");
        // serial_num(left_distance);
        // serial_string("cm) - Turning LEFT\n");
        sequenceStep = 1; // Move to left turn
        _delay_ms(500); // Brief pause
        return;
    }
    
    // Priority 2: Continue straight if left is blocked but front is clear
    if (front_distance > FRONT2_OBSTACLE_THRESHOLD_CM && front_distance != 0xFFFF) {
        // Continue moving straight with gyroscope and collision avoidance correction
        
        // Collision avoidance logic: steer away if too close to side obstacles
        float avoidance_correction = 0;
        
        // Check left sensor for collision avoidance
        if (left_distance != 0xFFFF && left_distance < COLLISION_AVOIDANCE_THRESHOLD) {
            // Too close to left obstacle - steer RIGHT (positive correction)
            float left_error = COLLISION_AVOIDANCE_THRESHOLD - left_distance;
            avoidance_correction += Kp_avoid * left_error;
            
            // serial_string("COLLISION AVOIDANCE: Left obstacle at ");
            // serial_num(left_distance);
            // serial_string("cm - steering RIGHT\n");
        }
        
        // Check right sensor for collision avoidance
        if (right_distance != 0xFFFF && right_distance < COLLISION_AVOIDANCE_THRESHOLD) {
            // Too close to right obstacle - steer LEFT (negative correction)
            float right_error = COLLISION_AVOIDANCE_THRESHOLD - right_distance;
            avoidance_correction -= Kp_avoid * right_error;
            
            // serial_string("COLLISION AVOIDANCE: Right obstacle at ");
            // serial_num(right_distance);
            // serial_string("cm - steering LEFT\n");
        }
        
        // Gyroscope correction for maintaining straight line
        float gyro_error = currentAngle - forwardSetpointAngle;
        int16_t gyro_correction = (int16_t)(Kp * gyro_error);
        
        // Combine collision avoidance and gyro corrections
        int16_t total_correction = gyro_correction + (int16_t)avoidance_correction;

        // Calculate new motor speeds with offset for the right motor
        int16_t rightSpeed = motor_speed + RIGHT_MOTOR_OFFSET + total_correction;
        int16_t leftSpeed = motor_speed - total_correction;

        // Clamp speeds to valid PWM range (0-255)
        if (rightSpeed > 255) rightSpeed = 255;
        if (rightSpeed < 0) rightSpeed = 0;
        if (leftSpeed > 255) leftSpeed = 255;
        if (leftSpeed < 0) leftSpeed = 0;

        set_right_motor_speed((uint8_t)rightSpeed);
        set_left_motor_speed((uint8_t)leftSpeed);
        return;
    }
    
    // Priority 3: Turn right if left and front are blocked but right is clear
    if (right_distance > WALL_DISTANCE_THRESHOLD_CM && right_distance != 0xFFFF) {
        _delay_ms(400);
        motor_stop();
        isMovingStraight = 0;
        serial_string("Front blocked, right space available (");
        serial_num(right_distance);
        serial_string("cm) - Turning RIGHT\n");
        sequenceStep = 3; // Move to right turn
        _delay_ms(500); // Brief pause
        return;
    }
    
    // Priority 4: Dead end - all sides blocked, turn right (180° turn logic)
    motor_stop();
    isMovingStraight = 0;
    serial_string("Dead end detected - All sides blocked! Turning RIGHT\n");
    serial_string("Distances: L=");
    serial_num(left_distance);
    serial_string(", F=");
    serial_num(front_distance);
    serial_string(", R=");
    serial_num(right_distance);
    serial_string("\n");
    sequenceStep = 5; // Move to dead end right turn (no forward movement after)
    _delay_ms(500); // Brief pause
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
// void motor_left() {
// 	PORTA &= ~((1 << MOTOR1_IN2) | (1 << MOTOR2_IN1));
//     PORTA |= (1 << MOTOR1_IN1) | (1 << MOTOR2_IN2);
//     set_right_motor_speed(motor_speed);     // Full speed right motor
//     set_left_motor_speed(motor_speed);  // Slower left motor
//     // _delay_ms(500);
// }


void motor_left() {
    serial_string("inside motor left");
	PORTA &= ~((1 << MOTOR1_IN2) | (1 << MOTOR2_IN1));
    PORTA |= (1 << MOTOR1_IN1) | (1 << MOTOR2_IN2);
    set_right_motor_speed(motor_speed);     // Full speed right motor
    set_left_motor_speed(motor_speed);  // Slower left motor
}

// Precise 90-degree turning functions
void start_left_turn() {
    startAngle = currentAngle;
    isTurning = 1;
    motor_left();  // Start turning left
    // _delay_ms(1000);
    serial_string("Starting LEFT turn from angle: ");
    serial_num((int16_t)startAngle);
    serial_string("°\n");
}

void start_right_turn() {
    startAngle = currentAngle;
    isTurning = 2;
    motor_right(); // Start turning right
    // _delay_ms(1000);
    serial_string("Starting RIGHT turn from angle: ");
    serial_num((int16_t)startAngle);
    serial_string("°\n");
}

void check_turn_completion() {
    if (isTurning == 1) { // Left turn
        float angleDiff = currentAngle - startAngle;
        
        if (angleDiff >= ANGLE_LEFT) {
            motor_stop();
            isTurning = 0;
            turnCounter++; // Increment turn counter
            // serial_string("LEFT turn complete! Final angle: ");
            // serial_num((int16_t)currentAngle);
            // serial_string("° Turn count: ");
            // serial_num(turnCounter);
            // serial_string("\n");
            _delay_ms(500); // Brief pause
            sequenceStep = 2; // Move to brief forward movement after left turn
            
            // Check if auto-recalibration is needed
            if (turnCounter >= RECALIBRATION_INTERVAL) {
                // Note: Function call will be updated in main() to pass actual variables
                sequenceStep = 8; // Special step for recalibration after left turn
                turnCounter = 0; // Reset counter
            }
        }
    }
    else if (isTurning == 2) { // Right turn
        float angleDiff = startAngle - currentAngle;
        
        if (angleDiff >= ANGLE_RIGHT) {
            motor_stop();
            isTurning = 0;
            turnCounter++; // Increment turn counter
            // serial_string("RIGHT turn complete! Final angle: ");
            // serial_num((int16_t)currentAngle);
            // serial_string("° Turn count: ");
            // serial_num(turnCounter);
            // serial_string("\n");
            _delay_ms(500); // Brief pause
            
            // Check if this was a dead end turn or normal right turn
            if (sequenceStep == 5) {
                // Dead end turn - go directly back to navigation
                sequenceStep = 0;
                serial_string("Dead end turn complete - resuming navigation\n");
            } else {
                // Normal right turn - do forward movement
                sequenceStep = 4; // Move to brief forward movement after right turn
            }
            
            // Check if auto-recalibration is needed
            if (turnCounter >= RECALIBRATION_INTERVAL) {
                // Note: Function call will be updated in main() to pass actual variables
                sequenceStep = 9; // Special step for recalibration after right turn
                turnCounter = 0; // Reset counter
            }
        }
    }
}

void execute_turn_sequence() {
    if (sequenceStep == 0 && !isTurning && !isMovingStraight) {
        // Start moving forward and continuously check for left-hand rule decisions
        motor_forward_straight();
        // serial_string("Moving forward continuously (left-hand rule active)...\n");
    }
    else if (sequenceStep == 1 && !isTurning) {
        // Make a 90° left turn
        start_left_turn();
    }
    else if (sequenceStep == 2 && !isTurning && !isMovingStraight) {
        // Distance-based forward movement after left turn with gyro assistance
        // serial_string("Moving forward 25cm after left turn with gyro assistance...\n");
        
        // Start gyro-assisted forward movement
        motor_forward_straight(); // This sets isMovingStraight = 1 and starts gyro correction
        
        // We'll use a new sequence step for distance monitoring
        sequenceStep = 6; // New step for distance-based forward movement after left turn
    }
    else if (sequenceStep == 6 && isMovingStraight) {
        // Monitor distance during forward movement after left turn
        uint16_t front_distance = sonar3_get_distance_cm();
        static uint16_t initial_front_distance_left = 0;
        static uint8_t distance_initialized_left = 0;
        
        if (!distance_initialized_left) {
            initial_front_distance_left = front_distance;
            distance_initialized_left = 1;
            // serial_string("Initial front distance after left: ");
            // serial_num(initial_front_distance_left);
            // serial_string("cm\n");
        }
        
        // Calculate how far we've moved
        uint16_t distance_moved = 0;
        if (initial_front_distance_left > front_distance && front_distance != 0xFFFF) {
            distance_moved = initial_front_distance_left - front_distance;
        }
        
        // Stop after moving ~25cm or if we detect an obstacle ahead
        if (distance_moved >= 20 || 
            (front_distance <= FRONT2_OBSTACLE_THRESHOLD_CM && front_distance != 0xFFFF)) {
            
            motor_stop();
            isMovingStraight = 0;
            distance_initialized_left = 0; // Reset for next time
            
            // serial_string("Forward movement after left complete. Moved: ");
            // serial_num(distance_moved);
            // serial_string("cm\n");
            
            sequenceStep = 0; // Reset to continuous navigation
            _delay_ms(500); // Brief pause before resuming
        }
    }
    else if (sequenceStep == 3 && !isTurning) {
        // Make a 90° right turn (normal right turn, not dead end)
        start_right_turn();
    }
    else if (sequenceStep == 4 && !isTurning && !isMovingStraight) {
        // Distance-based forward movement after right turn with gyro assistance
        serial_string("Moving forward 25cm after right turn with gyro assistance...\n");
        
        // Start gyro-assisted forward movement
        motor_forward_straight(); // This sets isMovingStraight = 1 and starts gyro correction
        
        // We'll use a new sequence step for distance monitoring
        sequenceStep = 7; // New step for distance-based forward movement after right turn
    }
    else if (sequenceStep == 7 && isMovingStraight) {
        // Monitor distance during forward movement after right turn
        uint16_t front_distance = sonar3_get_distance_cm();
        static uint16_t initial_front_distance_right = 0;
        static uint8_t distance_initialized_right = 0;
        
        if (!distance_initialized_right) {
            initial_front_distance_right = front_distance;
            distance_initialized_right = 1;
            serial_string("Initial front distance after right: ");
            serial_num(initial_front_distance_right);
            serial_string("cm\n");
        }
        
        // Calculate how far we've moved
        uint16_t distance_moved = 0;
        if (initial_front_distance_right > front_distance && front_distance != 0xFFFF) {
            distance_moved = initial_front_distance_right - front_distance;
        }
        
        // Stop after moving ~25cm or if we detect an obstacle ahead
        if (distance_moved >= 30 || 
            (front_distance <= FRONT2_OBSTACLE_THRESHOLD_CM && front_distance != 0xFFFF)) {
            
            motor_stop();
            isMovingStraight = 0;
            distance_initialized_right = 0; // Reset for next time
            
            serial_string("Forward movement after right complete. Moved: ");
            serial_num(distance_moved);
            serial_string("cm\n");
            
            sequenceStep = 0; // Reset to continuous navigation
            _delay_ms(500); // Brief pause before resuming
        }
    }
    else if (sequenceStep == 5 && !isTurning) {
        // Make a 90° right turn for dead end (no forward movement after)
        serial_string("Executing dead end right turn...\n");
        start_right_turn();
    }
    else if (sequenceStep == 8) {
        // Auto-recalibration needed after left turn
        serial_string("Triggering auto-recalibration after left turn...\n");
        sequenceStep = 2; // Return to normal sequence after recalibration
    }
    else if (sequenceStep == 9) {
        // Auto-recalibration needed after right turn  
        serial_string("Triggering auto-recalibration after right turn...\n");
        sequenceStep = 4; // Return to normal sequence after recalibration
    }
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

// Auto-recalibration function for gyroscope
void auto_recalibrate_gyro(float *AccErrorX_ptr, float *AccErrorY_ptr, 
                          float *GyroErrorX_ptr, float *GyroErrorY_ptr, float *GyroErrorZ_ptr,
                          float *gyroAngleX_ptr, float *gyroAngleY_ptr, float *yaw_ptr,
                          float *roll_ptr, float *pitch_ptr) {
    serial_string("Auto-recalibrating gyroscope after 2 turns...\n");
    motor_stop(); // Ensure robot is stationary
    _delay_ms(500); // Let it settle
    
    // Reset error values
    *AccErrorX_ptr = 0; *AccErrorY_ptr = 0;
    *GyroErrorX_ptr = 0; *GyroErrorY_ptr = 0; *GyroErrorZ_ptr = 0;
    
    // Quick calibration with 50 samples (faster than full calibration)
    for (uint16_t i = 0; i < 50; i++) {
        int16_t accelX, accelY, accelZ;
        int16_t gyroX, gyroY, gyroZ;
        
        MPU6050_ReadAccel(&accelX, &accelY, &accelZ);
        MPU6050_ReadGyro(&gyroX, &gyroY, &gyroZ);
        
        // Convert to proper units
        float AccX = accelX / 16384.0;
        float AccY = accelY / 16384.0;
        float AccZ = accelZ / 16384.0;
        float GyroX = gyroX / 131.0;
        float GyroY = gyroY / 131.0;
        float GyroZ = gyroZ / 131.0;
        
        // Sum accelerometer angles
        *AccErrorX_ptr += (atan(AccY / sqrt(AccX*AccX + AccZ*AccZ)) * 180.0 / M_PI);
        *AccErrorY_ptr += (atan(-AccX / sqrt(AccY*AccY + AccZ*AccZ)) * 180.0 / M_PI);
        
        // Sum gyroscope readings
        *GyroErrorX_ptr += GyroX;
        *GyroErrorY_ptr += GyroY;
        *GyroErrorZ_ptr += GyroZ;
        
        _delay_ms(5);
    }
    
    // Calculate average errors
    *AccErrorX_ptr = *AccErrorX_ptr / 50.0;
    *AccErrorY_ptr = *AccErrorY_ptr / 50.0;
    *GyroErrorX_ptr = *GyroErrorX_ptr / 50.0;
    *GyroErrorY_ptr = *GyroErrorY_ptr / 50.0;
    *GyroErrorZ_ptr = *GyroErrorZ_ptr / 50.0;
    
    // Reset angle tracking to prevent drift accumulation
    *gyroAngleX_ptr = 0; *gyroAngleY_ptr = 0; *yaw_ptr = 0;
    *roll_ptr = 0; *pitch_ptr = 0;
    
    serial_string("Gyro auto-recalibration complete!\n");
    serial_string("New gyro errors: X=");
    serial_num((int16_t)(*GyroErrorX_ptr * 10));
    serial_string(", Y=");
    serial_num((int16_t)(*GyroErrorY_ptr * 10));
    serial_string(", Z=");
    serial_num((int16_t)(*GyroErrorZ_ptr * 10));
    serial_string("\n");
    
    _delay_ms(200); // Brief pause before resuming
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
  
  // Initialize sonar sensors
  sonar_init();   // Left sonar (PD2, PD3)
  sonar2_init();  // Right sonar (PD4, PD5)
  sonar3_init();  // Front sonar (PB1, PB2)
  
	// Initialize LED pin (PB0) as output
	DDRB |= (1 << PB0);  // Set PB0 as output for LED
	// Note: PB1 is used for front sonar trigger, not for LED
	PORTB &= ~(1 << PB0); // Initially turn off LED
	
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

	serial_string("MPU6050 Left-Hand Rule Maze Solver Ready\n");
	serial_string("Press 'C' to calibrate (keep stationary!)\n");
	serial_string("Press 'G' to get angles and sensor data\n");
	serial_string("Press 'D' to test sonar sensors\n");
	serial_string("Wall detection threshold: ");
	serial_num(WALL_DISTANCE_THRESHOLD_CM);
	serial_string("cm\n");

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
		
		// Handle continuous obstacle avoidance if calibrated
		if (calibrated) {
			// Check for auto-recalibration trigger
			if (sequenceStep == 8 || sequenceStep == 9) {
				auto_recalibrate_gyro(&AccErrorX, &AccErrorY, &GyroErrorX, &GyroErrorY, &GyroErrorZ,
				                     &gyroAngleX, &gyroAngleY, &yaw, &roll, &pitch);
				// sequenceStep will be updated by execute_turn_sequence
			}
			
			if (isTurning) {
				check_turn_completion();
			} else if (isMovingStraight) {
          correct_straight_path();
      } else {
				execute_turn_sequence();
			}
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
				
				// Reset sequence to start from beginning
				sequenceStep = 0;
				isTurning = 0;
				
				calibrated = 1;
				serial_string("Calibration complete!\n");
				serial_string("Starting LEFT-HAND RULE navigation:\n");
				serial_string("1. Turn LEFT if space available (>20cm)\n");
				serial_string("2. Go STRAIGHT if left blocked, front clear\n");
				serial_string("3. Turn RIGHT if left/front blocked, right clear\n");
				serial_string("4. Turn RIGHT if all sides blocked (dead end)\n");
				
			} else if (cmd == 'G' || cmd == 'g') {
				if (calibrated) {
					serial_string("Current angle: ");
					serial_num((int16_t)currentAngle);
					serial_string("° | Roll: ");
					serial_num((int16_t)roll);
					serial_string("° | Step: ");
					serial_num(sequenceStep);
					serial_string(" | Turning: ");
					serial_num(isTurning);
					serial_string("\n");
				} else {
					serial_string("Please calibrate first (press 'C')\n");
				}
			} else if (cmd == 'D' || cmd == 'd') {
				// Test all sonar sensors
				uint16_t left_dist = sonar_get_distance_cm();
				uint16_t right_dist = sonar2_get_distance_cm();
				uint16_t front_dist = sonar3_get_distance_cm();
				
				serial_string("Sonar distances - Left: ");
				serial_num(left_dist);
				serial_string("cm, Right: ");
				serial_num(right_dist);
				serial_string("cm, Front: ");
				serial_num(front_dist);
				serial_string("cm");
				
				if (front_dist <= FRONT_OBSTACLE_THRESHOLD_CM && front_dist != 0xFFFF) {
					serial_string(" [OBSTACLE DETECTED!]");
				}
				serial_string("\n");
			}
		}
    
		_delay_ms(5); // 20Hz update rate
	}	
  
  return 0;
}