STM32F429 Flight Controller Core (IMU & AHRS System)

1. Project Overview

This project provides the Core Firmware for developing a custom Flight Controller based on the STM32F429VIT6 microcontroller. The system focuses on high-precision sensor data acquisition, signal processing, and Attitude Estimation (AHRS) specifically tuned for an F450 Drone frame.
The codebase is built with a modular architecture, integrating advanced filtering and Sensor Fusion algorithms to provide stable Pitch, Roll, Yaw, and Altitude data.

2. Hardware Architecture

Microcontroller (MCU): STM32F429VIT6 (ARM Cortex-M4, FPU).

Inertial Measurement Unit (IMU): ICM-20602 (6-DOF Gyroscope & Accelerometer) via SPI.

Magnetometer (Compass): Supports both QMC5883L and HMC5883L via I2C.

Barometer: BMP280 via I2C.


3. Key Features

A. Smart Sensor System

"Soft-Fail" Initialization: The system is designed to boot successfully even if individual sensors fail or are disconnected. The status of each sensor (Status_ICM, Status_Mag, Status_Baro) is monitored continuously to prevent system hangs.

Auto-Detect Magnetometer: The firmware automatically scans the I2C bus to detect and configure the appropriate driver for either QMC5883L (0x0D) or HMC5883L (0x1E) without requiring recompilation.

B. Algorithms & DSP (Digital Signal Processing)
Kalman Filter: Implements a 2-state Kalman Filter (Angle, Bias) to fuse Accelerometer and Gyroscope data, effectively eliminating mechanical vibration noise and Gyro drift.

Tilt-Compensated Compass: Calculates accurate Heading (Yaw) even when the drone is tilted, using rotation matrices derived from the Kalman Filter's Roll/Pitch estimates.

Hardware DLPF Configuration: Optimizes the Internal Digital Low Pass Filter of the ICM-20602 (Gyro ~41Hz, Accel ~44Hz) to match the vibration characteristics of standard drone frames.

Altitude Estimation: Calculates relative altitude based on atmospheric pressure data from the BMP280 using the standard barometric formula.

C. Calibration Routines
Gyroscope Calibration: Automatically samples 200 readings upon startup to calculate and remove static bias.

Magnetometer Calibration (Hard/Soft Iron): Features a dedicated calibration mode. By rotating the drone for ~15 seconds, the system records Min/Max values on all 3 axes to calculate Hard Iron Offset and Soft Iron Scale corrections.

D. Real-time Performance
Loop Rate: The main loop operates stably at approximately 200-250Hz.

Monitoring: Integrated "Heartbeat" LED on PG13 and global variables optimized for real-time debugging via STM32CubeIDE Live Watch.


4. Applications

This project serves as a foundational base for advanced flight control features such as:

Self-Leveling (Angle Mode).

Altitude Hold.

Inertial Navigation Systems (INS) for custom UAVs.
