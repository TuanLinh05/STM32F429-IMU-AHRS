/*
 * hmc5883l.c
 * Author: Vu Tuan Linh
 */

#include "hmc5883l.h"

// Biến hệ số Gain (mặc định +/- 1.3 Ga -> 1090 LSB/Gauss)
// Cần khớp với cấu hình trong Config_B
static float mgPerLSB = 0.92f; // Với Gain mặc định 1090

uint8_t HMC5883L_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check = 0;

    // 1. Kiểm tra thiết bị có online không
    if (HAL_I2C_IsDeviceReady(hi2c, HMC5883L_ADDR, 5, 100) != HAL_OK) {
        return 0; // Lỗi: Không tìm thấy cảm biến (hoặc sai địa chỉ do chip QMC)
    }

    // 2. Cấu hình CONFIG_A (0x00)
    // Bit 6-5 (MA): 11 (Average 8 samples) -> Giảm nhiễu
    // Bit 4-2 (DO): 110 (Data Rate 75Hz) -> Tốc độ cao cho Drone
    // Bit 1-0 (MS): 00 (Normal measurement)
    // Giá trị: 0x78
    uint8_t configA = 0x78;
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, HMC_CONFIG_A, 1, &configA, 1, 100);

    // 3. Cấu hình CONFIG_B (0x01) - Gain
    // Bit 7-5 (GN): 001 (Range +/- 1.3 Gauss, Resolution 0.92 mG/LSB)
    // Giá trị: 0x20
    uint8_t configB = 0x20;
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, HMC_CONFIG_B, 1, &configB, 1, 100);
    mgPerLSB = 0.92f; // Cập nhật hệ số tương ứng

    // 4. Cấu hình MODE (0x02)
    // Bit 1-0: 00 (Continuous-measurement mode)
    // Giá trị: 0x00
    uint8_t mode = 0x00;
    HAL_I2C_Mem_Write(hi2c, HMC5883L_ADDR, HMC_MODE, 1, &mode, 1, 100);

    return 1; // Thành công
}

void HMC5883L_Read(I2C_HandleTypeDef *hi2c, HMC5883L_t *magData) {
    uint8_t buffer[6];

    // Đọc 6 bytes bắt đầu từ thanh ghi 0x03
    // Lưu ý: HMC5883L tự động tăng địa chỉ thanh ghi
    HAL_I2C_Mem_Read(hi2c, HMC5883L_ADDR, HMC_DATA_OUT_X_MSB, 1, buffer, 6, 100);

    // Lưu ý thứ tự byte của HMC5883L là: X, Z, Y (Khác với các cảm biến thường là X, Y, Z)
    magData->X_Raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    magData->Z_Raw = (int16_t)((buffer[2] << 8) | buffer[3]);
    magData->Y_Raw = (int16_t)((buffer[4] << 8) | buffer[5]);

    // Chuyển đổi sang Gauss (miliGauss)
    magData->X = magData->X_Raw * mgPerLSB;
    magData->Y = magData->Y_Raw * mgPerLSB;
    magData->Z = magData->Z_Raw * mgPerLSB;

    // --- TÍNH TOÁN HEADING (CƠ BẢN) ---
    // Công thức này chỉ đúng khi cảm biến nằm phẳng (chưa bù nghiêng - Tilt Compensation)
    float heading = atan2f(magData->Y, magData->X);

    // Cộng góc lệch từ trường (Declination Angle)
    // Chuyển đổi độ sang radian: angle * (PI / 180)
    float declinationRad = MAG_DECLINATION * (3.14159f / 180.0f);
    heading += declinationRad;

    // Chuẩn hóa góc về 0 - 2PI
    if (heading < 0) heading += 2 * 3.14159f;
    if (heading > 2 * 3.14159f) heading -= 2 * 3.14159f;

    // Chuyển về độ
    magData->Heading = heading * 180.0f / 3.14159f;
}
