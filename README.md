# 🧭 STM32F429 Flight Controller Core – IMU & AHRS

![MCU](https://img.shields.io/badge/MCU-STM32F429%20(Cortex--M4F)-03234B?logo=stmicroelectronics&logoColor=white)
![IMU](https://img.shields.io/badge/IMU-ICM--20602-6A1B9A)
![Sensors](https://img.shields.io/badge/Sensors-QMC5883L%20%7C%20HMC5883L%20%7C%20BMP280-6A1B9A)
![Language](https://img.shields.io/badge/Language-C-555555?logo=c)

Core firmware for a **custom drone flight controller** on the **STM32F429**. It covers sensor acquisition, signal processing and **attitude & heading estimation (AHRS)**, tuned for an **F450 quadcopter frame**.
A modular sensor-fusion pipeline outputs stable **Roll, Pitch, Yaw and Altitude**.

---

## 🔧 Hardware

| Component | Interface | STM32F429 pins |
| :-- | :-- | :-- |
| **ICM-20602**: 6-DOF gyro + accel | SPI1 | PA5 SCK · PA6 MISO · PA7 MOSI · PA4 CS |
| **QMC5883L / HMC5883L**: magnetometer | I2C1 | PB6 SCL · PB7 SDA |
| **BMP280**: barometer | I2C1 | PB6 SCL · PB7 SDA |
| Heartbeat LED | GPIO | PG13 |

> Target MCU: STM32F429VIT6 flight controller. The CubeIDE project is configured for **STM32F429ZIT6** (STM32F429I-Discovery).

## ✨ Key features

### Smart sensor system
- **Soft-fail initialisation:** the system still boots if a sensor is missing or fails. `Status_ICM`, `Status_Mag` and `Status_Baro` are monitored continuously, so the firmware never hangs on a sensor.
- **Magnetometer auto-detection:** scans the I²C bus and selects the right driver for **QMC5883L (0x0D)** or **HMC5883L (0x1E)** without recompiling.

### Algorithms & DSP
- **Kalman filter:** a 2-state filter (angle, bias) fuses the accelerometer and gyroscope, rejecting vibration noise and gyro drift.
- **Tilt-compensated compass:** accurate heading even when the drone is tilted, using the Kalman roll and pitch.
- **Hardware DLPF:** the ICM-20602 internal low-pass filters are set to about 41 Hz (gyro) and 44 Hz (accel) to match typical frame vibration.
- **Altitude estimation:** relative altitude from BMP280 pressure, using the barometric formula.

### Calibration
- **Gyroscope:** 200 samples at start-up to remove static bias.
- **Magnetometer (hard & soft iron):** rotate the drone for about 15 s in calibration mode. Min/max values on each axis give the offset and scale corrections.

### Real-time performance
- Main loop runs at about **200–250 Hz**.
- Heartbeat LED on **PG13**. Global variables are laid out for easy debugging with **STM32CubeIDE Live Expressions**.

## 📂 Project structure

```text
ICM20602_F429_1132026/
├── Core/Inc/ICM20602.h, hmc5883l.h, IMU_System.h
├── Core/Src/ICM20602.c      # ICM-20602 SPI driver + DLPF configuration
├── Core/Src/hmc5883l.c      # Magnetometer driver (QMC5883L / HMC5883L)
├── Core/Src/IMU_System.c    # Sensor fusion: Kalman, tilt compensation, altitude, calibration
└── Core/Src/main.c          # Initialisation + main loop
```

## 🚀 Getting started

1. Open `ICM20602_F429_1132026` in **STM32CubeIDE**, build and flash it with an ST-Link.
2. Keep the board still during start-up for the gyro calibration.
3. Open **Live Expressions** to watch roll, pitch, yaw, altitude and the sensor status flags.

## 🛫 Applications

This firmware is the base for further flight-control features:
- **Self-levelling** (angle mode)
- **Altitude hold**
- **Inertial navigation** for custom UAVs

---

<p align="center">Made by <a href="https://github.com/TuanLinh05">Vu Tuan Linh</a> · HCMUT</p>
