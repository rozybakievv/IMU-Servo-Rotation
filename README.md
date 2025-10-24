# 🌀 IMU-Servo-Rotation
Embedded systems project based that maps the rotation (z-axis) of an MPU-6050 IMU (using I2C communication protocol) to the rotation of a servo controller in real-time.
Developped using PlatformIO on VS Code on an Arduino Nano without using any Arduino libraries, only direct register programming and timing.

## Hardware
- Arduino Nano ATmega328P microcontroller
- MPU6050	3-axis accelerometer and gyroscope sensor - register read continuously for data
- Servo Motor - controlled via PWM and angle mapped to IMU yaw value
- Breadboard, jumper wires

## Setup
1. Install PlatformIO extension on VS Code
2. Clone project and open as folder in VS Code
3. Build and upload: pio run --target upload
4. Open the serial monitor: pio device monitor

## How it works
- MPU6050 initialized via I2C communication protocol
- Raw gyro data read continuously from register access
- Z-axis angular velocity integrated over time (10ms) to estimate Yaw
- Servo motor angle mapped to estimated Yaw
- Integrated LED in Arduino Nano blinks if outside of rotation angle range (-79 degrees < range < 79 degrees)
