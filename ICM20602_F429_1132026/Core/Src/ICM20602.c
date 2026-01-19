/*
 * icm20602.c
 * Author: Vu Tuan Linh
 */

#include "ICM20602.h"

// Định nghĩa chân CS (Chip Select) - Cần sửa lại nếu bạn dùng chân khác
#define ICM_CS_PORT     GPIOA
#define ICM_CS_PIN      GPIO_PIN_4

// Hàm nội bộ điều khiển chân CS
static void ICM_Select() {
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_RESET);
}

static void ICM_Unselect() {
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
}

// Ghi 1 thanh ghi
static void ICM_WriteReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data) {
    uint8_t txData[2] = {reg & 0x7F, data}; // Bit 7 = 0 là Write
    ICM_Select();
    HAL_SPI_Transmit(hspi, txData, 2, 10);
    ICM_Unselect();
}

// Đọc 1 thanh ghi
static uint8_t ICM_ReadReg(SPI_HandleTypeDef *hspi, uint8_t reg) {
    uint8_t rxData = 0;
    uint8_t txData = reg | 0x80; // Bit 7 = 1 là Read
    ICM_Select();
    HAL_SPI_Transmit(hspi, &txData, 1, 10);
    HAL_SPI_Receive(hspi, &rxData, 1, 10);
    ICM_Unselect();
    return rxData;
}

// --- HÀM KHỞI TẠO ---
uint8_t ICM20602_Init(SPI_HandleTypeDef *hspi) {
    uint8_t whoami = ICM_ReadReg(hspi, ICM20602_WHO_AM_I);

    // Kiểm tra kết nối
    if (whoami != ICM20602_ID) {
        return 0; // Lỗi: Không tìm thấy cảm biến
    }

    // 1. Reset device
    ICM_WriteReg(hspi, ICM20602_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    // 2. Wake up & Auto select clock source
    ICM_WriteReg(hspi, ICM20602_PWR_MGMT_1, 0x01);
    HAL_Delay(10);

    // [Suy luận] Cấu hình tối ưu cho Drone F450:

    // 3. CONFIG: Gyro DLPF
    // Đặt DLPF = 3 (Bandwidth ~41Hz). Giúp lọc nhiễu rung động cơ khí.
    ICM_WriteReg(hspi, ICM20602_CONFIG, 0x03);

    // 4. ACCEL_CONFIG: Full Scale 8g
    // 0x10 = +/- 8g. Drone rung mạnh, dùng 2g hay 4g dễ bị bão hòa.
    ICM_WriteReg(hspi, ICM20602_ACCEL_CONFIG, 0x10);

    // 5. ACCEL_CONFIG2: Accel DLPF
    // Đặt DLPF = 3 (Bandwidth ~44.8Hz).
    ICM_WriteReg(hspi, ICM20602_ACCEL_CONFIG2, 0x03);

    // 6. GYRO_CONFIG: Full Scale 2000dps
    // 0x18 = +/- 2000 dps. Drone quay nhanh cần dải đo lớn.
    ICM_WriteReg(hspi, ICM20602_GYRO_CONFIG, 0x18);

    return 1; // Thành công
}

// --- HÀM ĐỌC DỮ LIỆU ---
void ICM20602_Read(SPI_HandleTypeDef *hspi, ICM20602_t *sensorData) {
    uint8_t buffer[14];
    uint8_t reg = ICM20602_ACCEL_XOUT_H | 0x80;

    // Đọc Burst 14 bytes: Accel(6) + Temp(2) + Gyro(6)
    ICM_Select();
    HAL_SPI_Transmit(hspi, &reg, 1, 10);
    HAL_SPI_Receive(hspi, buffer, 14, 10);
    ICM_Unselect();

    // Ghép byte (MSB trước)
    int16_t raw_ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t raw_ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t raw_az = (int16_t)((buffer[4] << 8) | buffer[5]);

    int16_t raw_temp = (int16_t)((buffer[6] << 8) | buffer[7]);

    int16_t raw_gx = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t raw_gy = (int16_t)((buffer[10] << 8) | buffer[11]);
    int16_t raw_gz = (int16_t)((buffer[12] << 8) | buffer[13]);

    // Chuyển đổi sang đơn vị vật lý
    // Scale Accel +/- 8g -> LSB Sensitivity = 4096 LSB/g
    sensorData->Ax = raw_ax / 4096.0f;
    sensorData->Ay = raw_ay / 4096.0f;
    sensorData->Az = raw_az / 4096.0f;

    // Scale Gyro +/- 2000dps -> LSB Sensitivity = 16.4 LSB/dps
    sensorData->Gx = raw_gx / 16.4f;
    sensorData->Gy = raw_gy / 16.4f;
    sensorData->Gz = raw_gz / 16.4f;

    sensorData->Temp = (raw_temp / 326.8f) + 25.0f;

    // --- TÍNH TOÁN GÓC PITCH / ROLL ---
    // Công thức Euler angles từ Accelerometer
    // Roll (trục X): Nghiêng trái/phải
    sensorData->Roll_Acc = atan2f(sensorData->Ay, sensorData->Az) * RAD_TO_DEG;

    // Pitch (trục Y): Chúi đầu/ngóc đầu
    // Dùng sqrt(Ay^2 + Az^2) để đảm bảo góc tính đúng khi drone nghiêng cả 2 trục
    sensorData->Pitch_Acc = atan2f(-sensorData->Ax, sqrtf(sensorData->Ay * sensorData->Ay + sensorData->Az * sensorData->Az)) * RAD_TO_DEG;
}

/* Thêm vào cuối file icm20602.c */

// Khởi tạo các tham số mặc định cho Kalman
void Kalman_Init(Kalman_t *kalman) {
    // [Suy luận] Các tham số này thường hoạt động tốt cho Drone F450
    // Q_angle: Độ tin cậy vào mô hình dự đoán (thấp = tin vào Gyro hơn)
    // R_measure: Độ nhiễu của phép đo (cao = lọc nhiễu Accel mạnh hơn nhưng trễ hơn)

    kalman->Q_angle = 0.001f;
    kalman->Q_bias  = 0.003f;
    kalman->R_measure = 0.03f;

    kalman->angle = 0.0f; // Reset góc
    kalman->bias  = 0.0f; // Reset bias

    // Reset ma trận hiệp phương sai P
    kalman->P[0][0] = 0.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 0.0f;
}

// Hàm tính toán góc mới
// newAngle: Góc đo từ Accelerometer (Noisy)
// newRate:  Vận tốc góc từ Gyroscope (Drifty) - Đơn vị: độ/giây (deg/s)
// dt:       Thời gian trôi qua giữa 2 lần đọc (giây)
float Kalman_GetAngle(Kalman_t *kalman, float newAngle, float newRate, float dt) {
    // 1. Dự đoán (Predict)
    kalman->rate = newRate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    // Cập nhật ma trận sai số dự đoán
    kalman->P[0][0] += dt * (dt*kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    // 2. Cập nhật (Update) dựa trên sai số đo lường
    float S = kalman->P[0][0] + kalman->R_measure; // Estimate Error

    // Tính Kalman Gain (K)
    float K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    // Tính sai lệch giữa dự đoán và thực tế đo được (Innovation)
    float y = newAngle - kalman->angle;

    // Hiệu chỉnh góc và bias
    kalman->angle += K[0] * y;
    kalman->bias  += K[1] * y;

    // Cập nhật lại ma trận hiệp phương sai sai số
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle;
}
