#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

#define BMP280_ADDRESS 0x76 // BMP280 I2C 地址

// 校准参数结构体
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

// 读取多个字节的数据
int readData(int fd, uint8_t reg, uint8_t *buf, int len) {
    if (write(fd, &reg, 1) != 1) {
        std::cerr << "写入寄存器地址失败" << std::endl;
        return -1;
    }
    if (read(fd, buf, len) != len) {
        std::cerr << "读取数据失败" << std::endl;
        return -1;
    }
    return 0;
}

// 读取校准参数
bool readCalibrationData(int fd, CalibParams &params) {
    uint8_t calib_data[24];
    if (readData(fd, 0x88, calib_data, 24) != 0) {
        return false;
    }

    params.dig_T1 = (calib_data[1] << 8) | calib_data[0];
    params.dig_T2 = (calib_data[3] << 8) | calib_data[2];
    params.dig_T3 = (calib_data[5] << 8) | calib_data[4];

    params.dig_P1 = (calib_data[7] << 8) | calib_data[6];
    params.dig_P2 = (calib_data[9] << 8) | calib_data[8];
    params.dig_P3 = (calib_data[11] << 8) | calib_data[10];
    params.dig_P4 = (calib_data[13] << 8) | calib_data[12];
    params.dig_P5 = (calib_data[15] << 8) | calib_data[14];
    params.dig_P6 = (calib_data[17] << 8) | calib_data[16];
    params.dig_P7 = (calib_data[19] << 8) | calib_data[18];
    params.dig_P8 = (calib_data[21] << 8) | calib_data[20];
    params.dig_P9 = (calib_data[23] << 8) | calib_data[22];

    return true;
}

// 配置传感器
bool configureSensor(int fd) {
    uint8_t ctrl_meas = 0x27; // 温度x1，气压x1，正常模式
    uint8_t reg = 0xF4;
    if (write(fd, &reg, 1) != 1 || write(fd, &ctrl_meas, 1) != 1) {
        return false;
    }
    return true;
}

// 读取原始温度数据并进行符号扩展
int32_t readTemperature(int fd) {
    uint8_t buf[3];
    if (readData(fd, 0xFA, buf, 3) != 0) {
        return 0;
    }
    int32_t temp_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    // 符号扩展20位到32位
    if (temp_raw & 0x00080000) {
        temp_raw |= 0xFFF00000;
    }
    return temp_raw;
}

// 读取原始气压数据并进行符号扩展
int32_t readPressure(int fd) {
    uint8_t buf[3];
    if (readData(fd, 0xF7, buf, 3) != 0) {
        return 0;
    }
    int32_t press_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    // 符号扩展20位到32位
    if (press_raw & 0x00080000) {
        press_raw |= 0xFFF00000;
    }
    return press_raw;
}

// 计算实际温度并返回fine_t用于气压计算
float compensateTemperature(int32_t temp_raw, const CalibParams &params, int32_t &fine_t) {
    int32_t var1 = ((((temp_raw >> 3) - ((int32_t)params.dig_T1 << 1))) * (int32_t)params.dig_T2) >> 11;
    int32_t var2 = (((((temp_raw >> 4) - (int32_t)params.dig_T1) * ((temp_raw >> 4) - (int32_t)params.dig_T1)) >> 12) * (int32_t)params.dig_T3) >> 14;
    fine_t = var1 + var2;
    return ((fine_t * 5 + 128) >> 8) / 100.0f;
}

// 计算实际气压
float compensatePressure(int32_t press_raw, int32_t fine_t, const CalibParams &params) {
    int64_t var1, var2, p;
    var1 = (int64_t)fine_t - 128000;
    var2 = var1 * var1 * (int64_t)params.dig_P6;
    var2 = var2 + ((var1 * (int64_t)params.dig_P5) << 17);
    var2 = var2 + ((int64_t)params.dig_P4 << 35);
    var1 = ((var1 * var1 * (int64_t)params.dig_P3) >> 8) + ((var1 * (int64_t)params.dig_P2) << 12);
    var1 = ((INT64_C(1) << 47) + var1) * (int64_t)params.dig_P1 >> 33;
    if (var1 == 0) {
        return 0; // 避免除以零
    }
    p = 1048576 - press_raw;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)params.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)params.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)params.dig_P7 << 4);
    return p / 25600.0f;
}

int main() {
    const char *i2c_dev = "/dev/i2c-1";
    int fd = open(i2c_dev, O_RDWR);
    if (fd < 0) {
        std::cerr << "无法打开 I2C 设备" << std::endl;
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE, BMP280_ADDRESS) < 0) {
        std::cerr << "无法与 BMP280 通信" << std::endl;
        close(fd);
        return -1;
    }

    CalibParams params;
    if (!readCalibrationData(fd, params)) {
        std::cerr << "读取校准参数失败" << std::endl;
        close(fd);
        return -1;
    }

    if (!configureSensor(fd)) {
        std::cerr << "配置传感器失败" << std::endl;
        close(fd);
        return -1;
    }

    sleep(1); // 等待传感器完成首次测量

    int32_t temp_raw = readTemperature(fd);
    int32_t press_raw = readPressure(fd);

    int32_t fine_t;
    float temperature = compensateTemperature(temp_raw, params, fine_t);
    float pressure = compensatePressure(press_raw, fine_t, params);

    std::cout << "温度: " << temperature << " °C" << std::endl;
    std::cout << "气压: " << pressure << " hPa" << std::endl;

    close(fd);
    return 0;
}