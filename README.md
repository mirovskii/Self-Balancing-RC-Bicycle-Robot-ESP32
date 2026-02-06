# Self-Balancing RC Bicycle Robot with ESP32 & PID Control

This project involves the design, construction, and control of a two-wheeled robotic bicycle that maintains its balance autonomously. The system utilizes an IMU sensor for tilt detection and a high-speed PID control loop to manage a steering servo for stabilization.

## Key Engineering Highlights
- **Controller:** ESP32 (Dual-core processor utilized for high-frequency control loops).
- **Control Theory:** Implemented a PID (Proportional-Integral-Derivative) control algorithm to maintain the bicycle at a specific target angle.
- **Sensor Fusion:** Utilized a Complementary Filter to combine data from the Accelerometer and Gyroscope (MPU6050), balancing long-term stability with short-term accuracy.
- **Actuators:** - SG90 Servo Motor: Active steering for balance.
  - N20 DC Motor & DRV8833: Rear-wheel propulsion.

## System Architecture
The robot maintains balance by adjusting its center of mass through precision steering maneuvers. The ESP32 reads tilt data via I2C, processes it through the PID filter, and outputs PWM signals to the steering servo.

[General Circuit Diagram](total_crt.png)

## Control Logic & Signal Processing
- **Sampling Time:** 2ms (500Hz) control loop for rapid response.
- **Complementary Filter:** `currentAngle = 0.66 * (prevAngle + gyroAngle) + 0.33 * (accAngle)`.
- **PID Tuning:** Implemented dynamic error calculation and sum constraints to prevent integral windup.

## Technical Analysis & Challenges
Engineering is an iterative process. During the testing phase, the following challenges were identified for future optimization:
- **PID Stability:** While the system achieves short-term balance, the PID constants require further fine-tuning to handle high-frequency oscillations.
- **Mechanical Center of Mass:** The 3D-printed frame's weight distribution impacted the servo's effective response range.
- **Vibration Noise:** High-frequency noise from the DC motor was observed in the IMU data, indicating a need for improved vibration damping or a more advanced Kalman Filter.

## Components
- ESP32 Development Board
- MPU6050 IMU Sensor
- DRV8833 Dual Motor Driver
- N20 DC Gear Motor
- SG90 Micro Servo

## Note on Collaboration
This is a group project developed for the **Robotics Course**.
