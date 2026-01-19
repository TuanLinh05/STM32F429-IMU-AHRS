#ifndef INC_IMU_SYSTEM_H_
#define INC_IMU_SYSTEM_H_

#include "stm32f4xx_hal.h"
#include <math.h>

// --- CẤU HÌNH ---
#define ICM_CS_PORT         GPIOA
#define ICM_CS_PIN          GPIO_PIN_4
#define RAD_TO_DEG          57.2957795f
#define DEG_TO_RAD          0.01745329f
#define MAG_DECLINATION     -1.0f

// --- STRUCT DATA ---
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} BMP280_Calib_t;

typedef struct {
    float Q_angle, Q_bias, R_measure;
    float angle, bias, P[2][2];
} Kalman_t;

// Struct chứa thông số hiệu chuẩn (NEW)
typedef struct {
    // Gyro Bias (Offset)
    float Gyro_X_Bias, Gyro_Y_Bias, Gyro_Z_Bias;

    // Accel Bias (Dùng để cân bằng mức - Leveling)
    float Accel_X_Bias, Accel_Y_Bias;

    // Mag Calibration (Hard Iron & Soft Iron)
    float Mag_X_Offset, Mag_Y_Offset, Mag_Z_Offset; // Hard Iron
    float Mag_X_Scale,  Mag_Y_Scale,  Mag_Z_Scale;  // Soft Iron
} IMU_CalibData_t;

typedef struct {
    uint8_t Status_ICM, Status_Mag, Status_Baro;
    float Ax, Ay, Az;
    float Gx, Gy, Gz;
    float Pitch, Roll;
    float Mx, My, Mz;
    float Yaw;
    float Pressure, Temperature, Altitude;
} IMU_Data_t;

// --- PROTOTYPES ---
uint8_t IMU_Init(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c);
void IMU_Update(SPI_HandleTypeDef *hspi, I2C_HandleTypeDef *hi2c, IMU_Data_t *data, float dt);

// Các hàm hiệu chuẩn mới
void IMU_Calibrate_Gyro(SPI_HandleTypeDef *hspi);
void IMU_Calibrate_Mag(I2C_HandleTypeDef *hi2c);

#endif /* INC_IMU_SYSTEM_H_ */
