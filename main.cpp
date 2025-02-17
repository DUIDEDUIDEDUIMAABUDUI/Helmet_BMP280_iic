#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>

#define BMP280_ADDRESS 0x76 // BMP280 默认 I2C 地址

// 读取寄存器数据
int read16(int fd, int reg) {
    uint8_t buf[2] = {static_cast<uint8_t>(reg)};
    if (write(fd, buf, 1) != 1) {
        std::cerr << "写入寄存器地址失败" << std::endl;
        return -1;
    }
    if (read(fd, buf, 2) != 2) {
        std::cerr << "读取数据失败" << std::endl;
        return -1;
    }
    return (buf[0] << 8) | buf[1];
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

    // 读取温度原始数据
    int temp_raw = read16(fd, 0xFA);
    
    // 读取气压原始数据
    int press_raw = read16(fd, 0xF7);
    
    // 打印原始数据
    std::cout << "原始温度数据: " << temp_raw << std::endl;
    std::cout << "原始气压数据: " << press_raw << std::endl;
    
    close(fd);
    return 0;
}