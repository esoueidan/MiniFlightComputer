# Mini Embedded Flight Computer (ESP32)

A lightweight flight controller simulation built with **PlatformIO** and **Wokwi**. This project implements a full sensor fusion and control loop for an ESP32-based drone.

## Features
- **Sensor Fusion**: Complementary Filter combining MPU6050 Accelerometer and Gyroscope data.
- **PID Control**: Real-time stabilization logic (Proportional, Integral, Derivative).
- **Non-blocking Architecture**: Multi-tasking heartbeat LED and sensor polling without `delay()`.
- **Simulation Ready**: Fully configured for VS Code Wokwi extension.

## Tech Stack
- **Hardware**: ESP32 (DevKit V1), MPU6050 (IMU).
- **Platform**: PlatformIO (Arduino Framework).
- **Simulator**: Wokwi.
- **Protocol**: I2C for sensor communication.

## Control Theory Implemented
### 1. Complementary Filter
We use a 98/2 ratio to get a stable angle:
$Angle = 0.98 \cdot (Angle + Gyro \cdot dt) + 0.02 \cdot Accel$

### 2. PID Regulation
The flight computer calculates motor output based on the error between the setpoint and the current angle:
- **P**: Corrects immediate error.
- **I**: Eliminates long-term drift (with Integral Windup protection).
- **D**: Dampens oscillations.

## How to Run
1. Install **VS Code** and the **PlatformIO** & **Wokwi** extensions.
2. Clone this repository.
3. Open the project in VS Code.
4. Press `F1` and select **Wokwi: Start Simulator**.
5. Open the Serial Monitor (115200 baud) to see the PID outputs and angles.

## Next Steps
- [x] I2C Sensor Interfacing
- [x] Complementary Filter implementation
- [x] PID Controller logic
- [ ] PWM Motor output (Simulation)
- [ ] Telemetry via Wi-Fi/Bluetooth
