#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

#define BMP280_ADDRESS 0x76

struct CalibParams {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
};

int readData(int fd, uint8_t reg, uint8_t *buf, int len) {
    if (write(fd, &reg, 1) != 1) {
        std::cerr << "写寄存器失败" << std::endl;
        return -1;
    }
    if (read(fd, buf, len) != len) {
        std::cerr << "读数据失败" << std::endl;
        return -1;
    }
    return 0;
}

bool readCalibrationData(int fd, CalibParams &params) {
    uint8_t calib_data[24];
    if (readData(fd, 0x88, calib_data, 24) != 0) return false;

    // 正确解析有符号值（符号扩展）
    params.dig_T1 = (calib_data[1] << 8) | calib_data[0];
    params.dig_T2 = (int16_t)(calib_data[3] << 8) | calib_data[2];
    params.dig_T3 = (int16_t)(calib_data[5] << 8) | calib_data[4];

    params.dig_P1 = (calib_data[7] << 8) | calib_data[6];
    params.dig_P2 = (int16_t)(calib_data[9] << 8) | calib_data[8];
    params.dig_P3 = (int16_t)(calib_data[11] << 8) | calib_data[10];
    params.dig_P4 = (int16_t)(calib_data[13] << 8) | calib_data[12];
    params.dig_P5 = (int16_t)(calib_data[15] << 8) | calib_data[14];
    params.dig_P6 = (int16_t)(calib_data[17] << 8) | calib_data[16];
    params.dig_P7 = (int16_t)(calib_data[19] << 8) | calib_data[18];
    params.dig_P8 = (int16_t)(calib_data[21] << 8) | calib_data[20];
    params.dig_P9 = (int16_t)(calib_data[23] << 8) | calib_data[22];

    return true;
}

bool configureSensor(int fd) {
    uint8_t config[2] = {0xF4, 0x27}; // 控制寄存器+模式
    if (write(fd, config, 2) != 2) return false;
    return true;
}

int32_t readTemperature(int fd) {
    uint8_t buf[3];
    if (readData(fd, 0xFA, buf, 3) != 0) return 0;
    
    // 正确符号扩展（20位）
    int32_t temp_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    temp_raw = temp_raw & 0x000FFFFF; // 确保20位
    if (temp_raw & 0x00080000) temp_raw |= 0xFFF00000;
    return temp_raw;
}

int32_t readPressure(int fd) {
    uint8_t buf[3];
    if (readData(fd, 0xF7, buf, 3) != 0) return 0;
    
    int32_t press_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    press_raw = press_raw & 0x000FFFFF;
    if (press_raw & 0x00080000) press_raw |= 0xFFF00000;
    return press_raw;
}

float compensateTemperature(int32_t temp_raw, const CalibParams &params, int32_t &fine_t) {
    int32_t var1 = ((((temp_raw >> 3) - ((int32_t)params.dig_T1 << 1))) * params.dig_T2;
    var1 >>= 11;
    
    int32_t var2 = ((((temp_raw >> 4) - params.dig_T1) * 
                   ((temp_raw >> 4) - params.dig_T1)) >> 12;
    var2 = (var2 * params.dig_T3) >> 14;
    
    fine_t = var1 + var2;
    float temp = (fine_t * 5.0f + 128.0f) / 256.0f;
    return temp / 100.0f;
}

float compensatePressure(int32_t press_raw, int32_t fine_t, const CalibParams &params) {
    int64_t var1, var2, p;

    var1 = (int64_t)fine_t - 128000LL;
    var2 = var1 * var1 * (int64_t)params.dig_P6;
    var2 += (var1 * (int64_t)params.dig_P5) << 17;
    var2 += (int64_t)params.dig_P4 << 35;
    
    var1 = ((var1 * var1 * (int64_t)params.dig_P3) >> 8) + 
           ((var1 * (int64_t)params.dig_P2) << 12);
    var1 = ((1LL << 47) + var1) * params.dig_P1 >> 33;

    if (var1 == 0) return 0;

    p = 1048576LL - press_raw;
    p = (((p << 31) - var2) * 3125LL) / var1;
    var1 = ((int64_t)params.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)params.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)params.dig_P7 << 4);

    return (float)p / 25600.0f; // 转换为hPa
}

int main() {
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0 || ioctl(fd, I2C_SLAVE, BMP280_ADDRESS) < 0) {
        std::cerr << "I2C初始化失败" << std::endl;
        return -1;
    }

    CalibParams params;
    if (!readCalibrationData(fd, params) || !configureSensor(fd)) {
        std::cerr << "初始化失败" << std::endl;
        close(fd);
        return -1;
    }

    // 调试输出校准参数
    std::cout << "T1:" << params.dig_T1 << " T2:" << params.dig_T2 
              << " T3:" << params.dig_T3 << std::endl;

    sleep(1); // 确保首次测量完成

    int32_t temp_raw = readTemperature(fd);
    int32_t press_raw = readPressure(fd);
    std::cout << "原始温度:" << temp_raw << " 原始气压:" << press_raw << std::endl;

    int32_t fine_t;
    float temp = compensateTemperature(temp_raw, params, fine_t);
    float press = compensatePressure(press_raw, fine_t, params);

    std::cout.precision(2);
    std::cout << "温度: " << std::fixed << temp << " °C\n"
              << "气压: " << press << " hPa" << std::endl;

    close(fd);
    return 0;
}