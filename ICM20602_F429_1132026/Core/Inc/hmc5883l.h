/*
 * hmc5883l.h
 * Author: Vu Tuan Linh
 */

#ifndef INC_HMC5883L_H_
#define INC_HMC5883L_H_

#include "stm32f4xx_hal.h"
#include <math.h>

// --- I2C ADDRESS ---
// Địa chỉ gốc 0x1E, dịch trái 1 bit để thành địa chỉ ghi (0x3C)
#define HMC5883L_ADDR       (0x1E << 1)

// --- REGISTER MAP ---
#define HMC_CONFIG_A        0x00
#define HMC_CONFIG_B        0x01
#define HMC_MODE            0x02
#define HMC_DATA_OUT_X_MSB  0x03
#define HMC_STATUS          0x09

// --- CALIBRATION ---
// Góc lệch từ trường tại Việt Nam (Magnetic Declination)
// Hà Nội ~ -1 độ, TP.HCM ~ -0.5 độ.
// Bạn có thể tra cứu chính xác theo tọa độ GPS.
#define MAG_DECLINATION     -1.0f

typedef struct {
    int16_t X_Raw, Y_Raw, Z_Raw; // Dữ liệu thô
    float X, Y, Z;               // Dữ liệu sau khi scale (Gauss)
    float Heading;               // Góc hướng mũi (0 - 360 độ)
} HMC5883L_t;

// Function Prototypes
uint8_t HMC5883L_Init(I2C_HandleTypeDef *hi2c);
void HMC5883L_Read(I2C_HandleTypeDef *hi2c, HMC5883L_t *magData);

#endif /* INC_HMC5883L_H_ */
