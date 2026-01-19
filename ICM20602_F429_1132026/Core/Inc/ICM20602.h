/*
 * icm20602.h
 * Author: Vu Tuan Linh
 */

#ifndef INC_ICM20602_H_
#define INC_ICM20602_H_

#include "stm32f4xx_hal.h"
#include <math.h>

// --- REGISTER MAP ---
#define ICM20602_WHO_AM_I       0x75
#define ICM20602_CONFIG         0x1A
#define ICM20602_GYRO_CONFIG    0x1B
#define ICM20602_ACCEL_CONFIG   0x1C
#define ICM20602_ACCEL_CONFIG2  0x1D // Accel DLPF
#define ICM20602_PWR_MGMT_1     0x6B
#define ICM20602_ACCEL_XOUT_H   0x3B // Bắt đầu vùng dữ liệu đo

#define ICM20602_ID             0x12

// --- CẤU HÌNH CHO DRONE (F450 Frame) ---
// DLPF: Lọc nhiễu rung động từ motor
// Gyro: 2000dps (để không bị bão hòa khi quay nhanh)
// Accel: 8g (để chịu được rung động và gia tốc lớn)
#define RAD_TO_DEG              57.2957795f

// Struct chứa toàn bộ dữ liệu
typedef struct {
    // Dữ liệu thô đã chuyển sang đơn vị vật lý (g và dps)
    float Ax, Ay, Az;
    float Gx, Gy, Gz;
    float Temp;

    // Góc nghiêng tính toán từ Accelerometer
    float Pitch_Acc;
    float Roll_Acc;
} ICM20602_t;

// Function Prototypes
uint8_t ICM20602_Init(SPI_HandleTypeDef *hspi);
void ICM20602_Read(SPI_HandleTypeDef *hspi, ICM20602_t *sensorData);

/* Thêm vào file icm20602.h */

typedef struct {
    float Q_angle;   // Process noise variance for the accelerometer
    float Q_bias;    // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance

    float angle;     // The angle calculated by the Kalman filter
    float bias;      // The gyro bias calculated by the Kalman filter
    float rate;      // Unbiased rate

    float P[2][2];   // Error covariance matrix
} Kalman_t;

// Function Prototypes mới
void Kalman_Init(Kalman_t *kalman);
float Kalman_GetAngle(Kalman_t *kalman, float newAngle, float newRate, float dt);
#endif /* INC_ICM20602_H_ */
