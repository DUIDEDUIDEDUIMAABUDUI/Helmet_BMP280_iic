#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

#define BMP280_ADDRESS 0x76 // BMP280 默认 I2C 地址

// 读取多个字节的数据
int readData(int fd, int reg, uint8_t *buf, int len) {
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

// 读取温度原始数据
int32_t readTemperature(int fd) {
    uint8_t buf[3];
    int reg = 0xFA;  // 温度数据寄存器地址
    if (readData(fd, reg, buf, 3) != 0) {
        return -1;
    }

    int32_t temp_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    return temp_raw;
}

// 读取气压原始数据
int32_t readPressure(int fd) {
    uint8_t buf[3];
    int reg = 0xF7;  // 气压数据寄存器地址
    if (readData(fd, reg, buf, 3) != 0) {
        return -1;
    }

    int32_t press_raw = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    return press_raw;
}

int main() {
    const char *i2c_dev = "/dev/i2c-1";
    int fd = open(i2c_dev, O_RDWR);
    if (fd < 0) {
        std::cerr << "无法打开 I2C 设备" << std::endl;
        return -1;
    }
    if (ioctl(fd, I2C_SLAVE, BMP280_ADDRESS) < 0) {
        std::cerr << "无法与 BMP280 设备通信" << std::endl;
        close(fd);
        return -1;
    }

    // 读取温度和气压原始数据
    int32_t temp_raw = readTemperature(fd);
    int32_t press_raw = readPressure(fd);

    // 打印原始数据
    std::cout << "原始温度数据: " << temp_raw << std::endl;
    std::cout << "原始气压数据: " << press_raw << std::endl;

    // 转换为实际温度和气压值 (需要校准参数，这里只是简单示例)
    // 校准公式会根据 BMP280 的数据手册来应用具体公式
    float temperature = temp_raw / 100.0f;  // 这里假设转换为摄氏度
    float pressure = press_raw / 100.0f;    // 这里假设转换为帕斯卡 (Pa)

    std::cout << "转换后的温度: " << temperature << " °C" << std::endl;
    std::cout << "转换后的气压: " << pressure << " Pa" << std::endl;

    close(fd);
    return 0;
}
