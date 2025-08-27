# 🤖 Automated Maze Solver - CSE 316 Project

[![ATmega32](https://img.shields.io/badge/Microcontroller-ATmega32-blue.svg)](https://www.microchip.com/en-us/product/ATmega32)
[![C++](https://img.shields.io/badge/Language-C++-green.svg)](https://isocpp.org/)
[![PlatformIO](https://img.shields.io/badge/IDE-PlatformIO-orange.svg)](https://platformio.org/)

> An intelligent robotic maze solver that uses sonar sensors and gyroscope-based navigation with the left-hand rule algorithm to autonomously navigate and solve mazes.

## 📋 Table of Contents
- [🎯 Project Overview](#-project-overview)
- [🔧 Hardware Components](#-hardware-components)
- [💻 Technologies Used](#-technologies-used)
- [🚀 Development Approach](#-development-approach)
- [🏗️ Maze Construction](#️-maze-construction)
- [🧠 Algorithm Implementation](#-algorithm-implementation)
- [⚙️ Setup & Installation](#️-setup--installation)
- [🛠️ Troubleshooting](#️-troubleshooting)
- [📈 Future Improvements](#-future-improvements)

## 🎯 Project Overview

This project demonstrates an autonomous maze-solving robot built for the **CSE-316 Microcontroller** course. The robot employs:

- **🧭 Left-hand rule algorithm** for maze navigation
- **📡 Triple ultrasonic sensor array** for obstacle detection  
- **🌐 MPU6050 gyroscope** for precise movement control
- **⚡ PID control** for straight-line movement

> **Note**: The current implementation can solve mazes where the exit is located at the boundary. Center-exit mazes require flood-fill algorithm implementation.

## 🔧 Hardware Components

### Essential Components

| Component | Quantity | Purpose |
|-----------|----------|---------|
| 🚗 Round Car Chassis (13cm) | 1 | Robot base platform |
| ⚙️ Universal Wheels | 2 | Omnidirectional movement |
| 🧭 MPU6050 Gyroscope | 1 | Orientation sensing |
| 🖥️ ATmega32 Microcontroller | 1 | Main processing unit |
| 🔌 Small Breadboard | 1 | Component connections |
| 📡 HC-SR04 Ultrasonic Sensors | 3 | Distance measurement |
| ⚡ L298N Motor Driver | 1 | Motor control |
| 📱 HC-05 Bluetooth Module | 1 | Wireless communication |
| 🔋 Rechargeable Batteries | 2 | Power supply |
| 🔘 Battery Holder with Switch | 1 | Power management |
| 📦 Cardboard Sheets | Several | Maze construction |
| 🔗 Jumper Wires (M-M, M-F) | Multiple | Connections |

### Additional Tools

| Tool | Purpose |
|------|---------|
| 🔥 Soldering Iron & Lead | Permanent connections |
| ⚡ Buck Converter | Voltage regulation |
| 👁️ IR Sensors (Optional) | Additional obstacle detection |
| 🌡️ Heat Shrink Tubing | Wire management |
| 📏 Multimeter | Circuit debugging |
| 💾 USB Programmer | Code uploading |

## 💻 Technologies Used

- **🖥️ PlatformIO** - Development environment
- **📝 C++** - Programming language  
- **🔧 VS Code** - Code editor
- **📡 Serial Communication** - Bluetooth debugging
- **⚙️ Timer-based PWM** - Motor speed control

## 🚀 Development Approach

### Phase 1: 🏗️ Chassis Assembly

The maze solver is a very challenging project, especially with the ATmega32, so it's better to break it down into manageable chunks of work.

#### 🚗 Chassis Selection & Motor Setup
- **Compact wiring design** for easy maintenance and future debugging
- **Proper motor selection** - ensure voltage compatibility between all motors
- **Avoid rectangular 4-wheel chassis** - they are not ideal for maze solvers as they can't make in-place turns
- **Motor compatibility warning** ⚠️ - Similar looking motors can require different voltage levels, so use motors with identical specifications

> 🔥 **Critical Warning**: When running 4 motors with a single L298N motor driver (designed for 2 motors), be extremely careful. We destroyed 2 L298N motor drivers, USB burners, and even ATmega32 microcontrollers in the process!

#### 🔌 Wiring & Power Management
- **Wire management** - Crucial in this step as debugging will happen frequently
- **Switch system implementation** - Highly recommended for easy power control (see reference image)
- **Power distribution planning** - Essential for stable operation

### Phase 2: 📱 Bluetooth Remote Control

Bluetooth implementation is particularly challenging for movable objects not directly connected to a laptop. For stationary projects, Bluetooth is much simpler.

#### ⚡ Power Supply Challenges
- **Separate power supply for Bluetooth** - Critical for stable connection
- **Power level fluctuation issue** - When the car moves or other sensors use the same battery, power levels decrease and disconnect Bluetooth
- **Recommended power solutions**:
  - 🔋 **5V separate source** - Most reliable option
  - 🔋 **3.7V Li-Po batteries** - Good alternative
  - 🔋 **9V batteries with buck converters** - Convert to 5V using cheap buck converters

#### 🔧 Connection Requirements
- **Using serial.h** - Makes Bluetooth connection much easier
- **Without serial.h** - Requires precision:
  - 3.7V to ATmega power
  - All grounds must be connected
  - Bluetooth needs 5V power supply
- **Debugging capability** - Bluetooth later becomes essential for console outputs, error checking, and device calibration

### Phase 3: ⚡ Motor Control Implementation

#### 🎛️ PWM Timer Setup
Essential for PID control implementation:

```cpp
// Timer0 & Timer2 PWM Configuration
// Speed range: 0-255 
// Operational minimum: 80 (below this motors won't move)
// Base speed: 120 (chosen for stability)
// Left/Right motor speed tuning required for balance
```

#### ⚖️ Motor Calibration Challenges
- **Speed equalization** - Left and right motors need individual tuning
- **Hardware limitations** - Cheap motors were our biggest problem
- **Gyroscope necessity** - Required for long-distance straight movement due to motor inconsistencies

### Phase 4: 🧭 Gyroscope Integration

At this stage, we confirmed all car components work, but cheap motors created the biggest challenge requiring gyroscope assistance.

#### 📐 Angle Detection & Calibration
- **90-degree turn accuracy** - Essential for maze navigation
- **Angle detection precision** - Gyroscope can be tricky; accurate angle measurement is crucial
- **Calibration requirements** - Proper setup needed for reliable angle readings

#### 🎯 PID Implementation for Straight Movement
```cpp
// PID Configuration for straight movement
// Used only P (Proportional) control - sufficient for basic operation
// D (Derivative) term could improve robustness in some situations
// Complementary filter: 96% gyro + 4% accelerometer
```

#### 🔄 Turn Implementation
- **90-degree turns** - With proper motors, hardcoded angles work well
- **PID for turning** - Not implemented but could improve precision
- **Angle-based control** - Sufficient for most maze-solving scenarios

### Phase 5: 📡 Sonar Sensor Tuning

#### 🎯 Calibration Process
```cpp
// Sonar calibration for exact cm values
// Multiplication factor: 2.3/10
// Individual sensor calibration required
```

#### ⏱️ Timing Constraints & Performance Impact
- **No timer implementation** - Used functions to call and get values when needed
- **Critical performance issue**:
  - Each sonar read: ~30ms
  - 3 sonar readings: 90ms total
  - **Loop frequency impact**: While(1) loop called ~10 times/second instead of target 200 times/second (every 5ms)
  - **Gyroscope effectiveness reduced** - Slow main loop makes gyro-assisted straight movement less effective

#### 🚫 Sonar-based PID Challenges
- **Attempted implementation** - Sonar-based PID tested but unsuccessful
- **Root cause** - Sonars make the main loop too slow for effective PID control
- **Performance trade-off** - Accuracy vs. response time balance needed

## 🏗️ Maze Construction

Building the maze requires careful consideration of dimensions and sensor limitations.

### 📐 Optimal Dimensions

#### 🎯 Recommended Specifications
- **Cell Size**: 35-37.5cm × 35-37.5cm *(ideal after testing)*
- **Wall Height**: 15-20cm
- **Robot Size**: 13-15cm diameter

#### 📏 Dimension Testing & Lessons Learned

**Initial Attempt: 30cm × 30cm**
- **Result**: ❌ Too small for reliable operation
- **Issues**: Required extreme precision that our hardware couldn't achieve
- **Robot size**: 13-15cm in 30cm cells left very little margin for error

**Second Attempt: 45cm × 45cm**  
- **Result**: ❌ Too large, caused sensor accuracy problems
- **Issues**: Sonar-assisted straight movement became unpredictable
- **Problem**: Exceeded effective sonar range capabilities

**Final Solution: 35-37.5cm × 35-37.5cm**
- **Result**: ✅ Optimal balance between precision and sensor capability
- **Benefits**: Sufficient space for navigation while maintaining sensor accuracy

### 🎯 Design Considerations

#### 📡 Sonar Range Limitations
- **Effective Range**: 50-60cm maximum for accurate detection
- **Accuracy Degradation**: Beyond this range, distance readings become highly unpredictable
- **Distance Fluctuation**: Large variations break conditional logic in navigation algorithm

#### 🔧 Construction Guidelines
- **Wall Material**: Solid surfaces for reliable sonar reflection
- **Wall Thickness**: Sufficient to prevent false readings through walls
- **Corner Construction**: Clean 90-degree angles for proper turning
- **Surface Texture**: Smooth walls for consistent sonar response

#### ⚠️ Critical Size Considerations
- **Too Small (≤30cm)**: Requires hardware precision beyond ATmega32 capabilities
- **Too Large (≥45cm)**: Causes sonar accuracy issues and unpredictable navigation
- **Sweet Spot (35-37.5cm)**: Perfect balance for reliable autonomous navigation

## 🧠 Algorithm Implementation

In this phase, we developed an algorithm capable of actually solving the maze. Due to ATmega32 constraints (memory limitations and debugging challenges with unreliable Bluetooth), we chose the simplest effective approach: the Left-Hand Rule algorithm.

### 🎯 Algorithm Selection Rationale

#### � Memory Constraints
- **ATmega32 limitations** - Limited memory for complex pathfinding algorithms
- **Debugging challenges** - Bluetooth reliability issues made complex debugging difficult
- **Simplicity requirement** - Need for straightforward, reliable algorithm

#### �🔄 Left-Hand Rule Advantages
- **Memory efficient** - No need to store maze map
- **Simple implementation** - Easy to code and debug
- **Guaranteed solution** - Will find exit in simply connected mazes
- **Robust operation** - Less prone to sensor noise and timing issues

### 🔄 Left-Hand Rule Logic Flow

```text
Start Position
     ↓
Check Left Side
     ↓
Space Available? ──YES──→ Turn Left (200-300ms delay) ──→ Move Forward One Cell
     ↓ NO
Check Front
     ↓
Space Available? ──YES──→ Move Straight with Gyro Assistance ──→ Continue until obstacle/turn opportunity
     ↓ NO                      ↑
Check Right Side               │
     ↓                        │
Space Available? ──YES──→ Turn Right ──→ Move Forward One Cell ──┘
     ↓ NO
Dead End Detected
     ↓
Turn Right Only (No forward movement)
```

### 📝 Detailed Algorithm Steps

#### 1. 🔍 Left Priority Check
```cpp
// Left side sensor reading
if (left_distance > SAFE_DISTANCE) {
    turn_left();           // 200-300ms turning delay
    delay(200-300);        // Wait for turn completion
    move_one_cell();       // Sonar-guided single cell movement
    return;                // Restart algorithm cycle
}
```

**Why Left Priority?**
- **Left-hand rule requirement** - Always prefer left turns to follow wall
- **Delay necessity** - 200-300ms ensures complete turn before movement
- **Single cell movement** - Precise navigation using sonar feedback

#### 2. ➡️ Forward Movement Check
```cpp
// Front sensor reading
if (front_distance > SAFE_DISTANCE) {
    // Gyro-assisted straight movement
    move_straight_with_gyro();
    // Continue until:
    // - Left space becomes available (higher priority)
    // - Obstacle detected ahead (20cm threshold)
    // - Wall proximity triggers turn decision
}
```

**Straight Movement Details:**
- **Gyro assistance essential** - Compensates for motor inconsistencies
- **Continuous monitoring** - Check for left opportunities while moving
- **Distance threshold** - Stop at 20cm from obstacles
- **PD control integration** - Maintains straight-line trajectory

#### 3. 🔄 Right Turn Fallback
```cpp
// Right side sensor reading  
if (right_distance > SAFE_DISTANCE) {
    turn_right();          // Immediate turn (no delay like left)
    move_one_cell();       // Sonar-guided movement
    return;                // Restart algorithm cycle
}
```

**Right Turn Characteristics:**
- **No additional delay** - Left and front already blocked
- **Immediate execution** - Faster response than left turns
- **Same movement pattern** - Single cell advancement

#### 4. 🚫 Dead End Handling
```cpp
// All sides blocked - dead end detected
if (left_blocked && front_blocked && right_blocked) {
    turn_right();          // Single right turn only
    // NO forward movement - just reorient
    // Next cycle will reassess available paths
}
```

**Dead End Logic:**
- **Orientation only** - Turn right without advancing
- **No movement** - Prevents collision with walls
- **Algorithm restart** - Next iteration finds available path
- **Backtracking effect** - Eventually leads to alternate routes

### 🎛️ Movement Implementation Details

#### � Turn Execution
```cpp
// Left turn implementation
void turn_left() {
    // Stop all motors
    motor_stop();
    
    // Execute 90-degree left turn using gyroscope
    target_angle = current_angle - 90;
    while (abs(current_angle - target_angle) > ANGLE_TOLERANCE) {
        // Turn left motors
        rotate_left();
        update_gyro_reading();
    }
    
    // Stop and stabilize
    motor_stop();
    delay(200-300);  // Stabilization delay
}
```

#### 📏 Single Cell Movement
```cpp
// Sonar-guided single cell movement
void move_one_cell() {
    target_distance = CELL_SIZE;  // 35-37.5cm
    distance_traveled = 0;
    
    while (distance_traveled < target_distance) {
        // Move forward with gyro assistance
        motor_forward_straight();  // PD control for straight movement
        
        // Monitor front obstacle
        if (front_sensor() < OBSTACLE_THRESHOLD) {
            break;  // Stop if obstacle detected
        }
        
        // Update distance (wheel encoders or time-based)
        distance_traveled += calculate_distance_increment();
    }
    
    motor_stop();
}
```

#### 🧭 Gyro-Assisted Straight Movement
```cpp
// Continuous straight movement with gyro correction
void move_straight_with_gyro() {
    while (true) {
        // PD control for straight movement
        error = target_angle - current_angle;
        derivative = error - previous_error;
        
        correction = (Kp * error) + (Kd * derivative);
        
        // Apply motor speed corrections
        left_speed = BASE_SPEED - correction;
        right_speed = BASE_SPEED + correction;
        
        set_motor_speeds(left_speed, right_speed);
        
        // Check exit conditions
        if (left_sensor() > SAFE_DISTANCE) return;    // Left opportunity
        if (front_sensor() < STOP_DISTANCE) return;   // Obstacle ahead
        
        previous_error = error;
        delay(CONTROL_PERIOD);  // 50ms control loop
    }
}
```

### 🔍 Sensor Integration & Timing

#### ⏱️ Timing Considerations
- **Sensor reading time**: 30ms per ultrasonic sensor
- **Total sensor delay**: 90ms for 3 sensors (Front, Left, Right)
- **Loop frequency impact**: Reduces from target 200Hz to ~10Hz
- **Gyro effectiveness**: Reduced due to slow main loop

#### 📊 Decision Thresholds
```cpp
#define SAFE_DISTANCE     25    // cm - minimum space for movement
#define OBSTACLE_THRESHOLD 20   // cm - stop distance from obstacles  
#define STOP_DISTANCE     15    // cm - emergency stop threshold
#define ANGLE_TOLERANCE   2     // degrees - turn accuracy
#define CELL_SIZE         35    // cm - maze cell dimension
```

### 🚫 Algorithm Limitations & Trade-offs

#### ⚠️ Known Issues
- **Sonar loop delay** - 90ms sensor reading reduces PD control effectiveness
- **Simple pathfinding** - No optimization for shortest path
- **Memory vs. speed** - Chose simplicity over advanced algorithms
- **Sensor noise** - Basic filtering, susceptible to environmental factors

#### 🎯 Design Decisions
- **Reliability over speed** - Prioritized consistent operation
- **Hardware limitations** - Worked within ATmega32 constraints  
- **Debugging practicality** - Simple algorithm easier to troubleshoot
- **Proven approach** - Left-hand rule is time-tested for maze solving





## ⚙️ Setup & Installation

### 📋 Prerequisites

Before starting, ensure you have:

- **PlatformIO IDE** (VS Code extension)
- **ATmega32 microcontroller**  
- **Complete hardware components** (see [Hardware Components](#-hardware-components))
- **USB ASP Burner** for code uploading

### 🛠️ Software Setup

#### 1. Development Environment Setup

**Download and Install:**
- **VS Code** - Primary code editor
- **PlatformIO extension** - ATmega32 development support

#### 2. PlatformIO Project Configuration

```bash
# Install PlatformIO Core
pip install platformio

# Create new project for ATmega32
pio project init --board ATmega32 --project-dir .
```

#### 3. Board Configuration in platformio.ini

Copy these contents into your `platformio.ini` file:

```ini
[env:ATmega32]
platform = atmelavr
board = ATmega32
framework = arduino
upload_port = /dev/ttyUSB0  # Adjust for your system
monitor_speed = 9600
upload_protocol = usbasp
```

### 🔧 Code Upload Process

#### 📝 Initial Code Upload

1. **Copy code** into `src/main.cpp`
2. **Build project**: Use PlatformIO build command
3. **Upload via USB Burner** to ATmega32

#### ⚠️ Common Upload Issues & Solutions

**Problem 1: Upload Failure**
- **Cause**: Port allocation issues with USB burner
- **Solution**: Use the provided `run.sh` script (see below)

**Problem 2: Code vs Upload Errors**
```text
🔍 Error Types:
1) Code Error ❌
   - Compiler detects syntax/logic errors
   - Must fix code before upload attempt
   
2) Upload Error ❌  
   - Hardware connection issues
   - Most difficult to resolve
   - Solution: Use run.sh script
```

#### 🛠️ Upload Script Solution

**run.sh Script** (Essential for reliable uploads):

```bash
#!/bin/bash
cd .pio/build/atmega32
rm -rf firmware.hex firmware.elf
cd ../../..
pio run
avrdude -c usbasp -p m32 -e  
pio run --target upload
```

**Script Usage:**
```bash
# Make script executable
chmod +x run.sh

# Run upload script  
./run.sh
```

> 📝 **Note**: Initial upload may work directly, but subsequent uploads typically require the `run.sh` script due to USB burner and port allocation issues.

#### 🔧 Upload Troubleshooting

**Step-by-Step Debugging:**

1. **Clean src folder** - Remove everything except `main.cpp`
2. **Test with simple code** - Try `atmega_test.cpp` first
3. **Check connections** - Verify USB burner to ATmega32 wiring
4. **Ignore SCK warnings** - Normal as long as compilation and upload succeed

**Warning Messages (Safe to Ignore):**
```text
⚠️ SCK Clock Warning
- Common during upload process
- Safe to ignore if code compiles and uploads successfully
- Does not affect functionality
```

### 🔌 Hardware Assembly Guide

#### Step 1: 🏗️ Chassis & Basic Assembly

1. **Choose appropriate chassis** - Avoid rectangular 4-wheel designs
2. **Motor installation** - Ensure all 4 motors have identical specifications
3. **Power switch system** - Essential for easy debugging
4. **Cable management** - Organize for future troubleshooting access

#### Step 2: 🔌 Electronics Integration

**ATmega32 Setup:**
- Mount on breadboard or PCB
- Connect 16MHz crystal oscillator
- Add required capacitors and resistors

**L298N Motor Driver Connection:**
⚠️ **Critical Warning**: L298N drivers are fragile! We destroyed 2 drivers, USB burners, and ATmega32 units during testing.

**Power Management:**
```text
ATmega32: 5V regulated supply
Motors: 6-12V (via L298N)  
Bluetooth: Separate 5V source (CRITICAL)
Sensors: 5V from main supply
```

#### Step 3: 📡 Sensor Mounting

**Ultrasonic Sensor Positioning:**
- **Front sensor** - Obstacle detection
- **Left sensor** - Left-hand rule implementation  
- **Right sensor** - Alternative path detection
- **Mounting height** - Consistent with maze wall height

**MPU6050 Gyroscope:**
- **Stable mounting** - Minimize vibration
- **I2C connection** - SDA/SCL to ATmega32
- **Calibration access** - Easy positioning for initial setup

#### Step 4: 🔗 Detailed Wiring Connections

| Component | ATmega32 Pin | Voltage | Notes |
|-----------|--------------|---------|-------|
| **Motors (Left)** | Timer0 PWM | 6-12V | Via L298N driver |
| **Motors (Right)** | Timer2 PWM | 6-12V | Via L298N driver |
| **MPU6050** | I2C (SDA/SCL) | 5V | Gyroscope/Accelerometer |
| **HC-05 Bluetooth** | UART (RX/TX) | 5V | **Separate power supply!** |
| **Ultrasonic Front** | Digital I/O | 5V | Trigger/Echo pins |
| **Ultrasonic Left** | Digital I/O | 5V | Trigger/Echo pins |
| **Ultrasonic Right** | Digital I/O | 5V | Trigger/Echo pins |
| **Buzzer** | Digital Pin | 5V | Status indication |

### 🚨 Advanced Troubleshooting

#### Hardware Issues

| Problem | Symptoms | Solution |
|---------|----------|----------|
| **L298N Failure** | Motors not responding | Check wiring, replace driver |
| **Bluetooth Disconnection** | Connection drops during movement | Use separate 5V supply |
| **Motor Speed Mismatch** | Car doesn't go straight | Calibrate individual motor speeds |
| **Gyroscope Drift** | Inaccurate turns | Recalibrate, check mounting stability |
| **Sonar False Readings** | Erratic distance measurements | Check wiring, adjust calibration factor |

#### Software Issues

| Problem | Symptoms | Solution |
|---------|----------|----------|
| **Upload Fails** | avrdude errors | Use `run.sh` script |
| **Code Compilation Error** | Build fails | Check syntax, includes |
| **Serial Monitor Silent** | No output | Verify baud rate (9600) |
| **Bluetooth Not Connecting** | Can't pair/connect | Check power, ground connections |
| **PID Oscillation** | Unstable straight movement | Tune Kp, Kd values |

#### Performance Optimization

**Critical Timing Issues:**
- **Sonar delay**: 90ms total (30ms × 3 sensors)
- **Target loop frequency**: 200Hz (5ms intervals)  
- **Actual loop frequency**: ~10Hz due to sonar delays
- **Impact**: Reduced gyroscope-assisted movement effectiveness

**Optimization Strategies:**
```cpp
// Reduce sensor reading frequency
if (sensor_counter % 3 == 0) {
    read_all_sensors();  // Only every 3rd loop
}

// Implement sensor scheduling
switch (sensor_index) {
    case 0: read_front_sensor(); break;
    case 1: read_left_sensor(); break;  
    case 2: read_right_sensor(); break;
}
sensor_index = (sensor_index + 1) % 3;
```

### 📊 Monitoring & Debugging

#### Serial Monitor Setup

```bash
# Monitor via USB (when connected to computer)
pio device monitor --baud 9600

# Monitor via Bluetooth (wireless debugging)
# Connect HC-05 to phone/computer Bluetooth
# Use serial terminal app
```

#### Real-time Debugging Commands

| Command | Function | Output |
|---------|----------|--------|
| `S` | Start movement | Begins PD control |
| `T` | Stop movement | Halts all motors |
| `P` | Show PD values | Current Kp, Kd, error values |
| `G` | Display angles | Gyroscope readings |
| `C` | Calibrate gyro | Reset angle reference |

#### Debugging Best Practices

1. **Power sequence** - Turn on car AFTER starting serial monitor
2. **Bluetooth backup** - Use wireless monitoring for moving tests
3. **Parameter tuning** - Adjust PID values via real-time commands
4. **Sensor validation** - Monitor distance readings during operation

## 🎯 Usage Instructions

### 1. 🚀 Initial Setup

```cpp
// Upload main.cpp for autonomous operation
// Auto-calibration starts immediately
// Robot begins maze solving after 600ms
```

### 2. 🎮 Manual Control (store3.cpp)

| Command | Action |
|---------|--------|
| `S` | Start PD control movement |
| `T` | Stop all movement |
| `P` | Display PD control values |
| `G` | Show current gyroscope angles |

### 3. 📊 Monitoring

```bash
# Real-time debugging via Bluetooth
pio device monitor --baud 9600

# Watch PD control values
# Monitor gyroscope calibration
# Track sensor readings
```

## 🚀 Future Improvements

### 🎯 Planned Enhancements

- [ ] **🔄 Flood Fill Algorithm** - Advanced pathfinding for optimal routes
- [ ] **🏎️ Enhanced Motors** - Higher precision and reliability
- [ ] **📡 Better Ultrasonic Sensors** - Improved range and accuracy
- [ ] **🎛️ PID for Turning** - Smoother and more accurate rotations
- [ ] **📊 Sonar-based PID** - Enhanced straight-line movement
- [ ] **🖨️ PCB Design** - Professional circuit board layout

### 💡 Technical Improvements

| Feature | Current | Target |
|---------|---------|--------|
| **Control Algorithm** | PD Controller | Full PID + Kalman Filter |
| **Pathfinding** | Left-hand Rule | Flood Fill + A* |
| **Sensors** | HC-SR04 | LIDAR/ToF sensors |
| **Processing** | ATmega32 | ESP32/Raspberry Pi |

## 🤝 Contributing

We welcome contributions! Please feel free to:

- 🐛 **Report bugs** and issues
- 💡 **Suggest improvements** and new features
- 🔧 **Submit pull requests** with enhancements
- 📖 **Improve documentation** and examples

## 📄 License

This project is open source and available under the [MIT License](LICENSE).

## 🙏 Acknowledgments

- **CSE 316 Course** - Academic foundation and guidance
- **ATmega32 Community** - Hardware support and documentation
- **PlatformIO Team** - Excellent development environment
- **Open Source Contributors** - Libraries and code examples

---

## 🤖 Project Summary

Built with passion for autonomous robotics and maze solving algorithms 🧠

*Star ⭐ this repository if you found it helpful!*


