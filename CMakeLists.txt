cmake_minimum_required(VERSION 3.10)
project(BPM_Project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


file(GLOB SOURCES "*.cpp")

add_executable(BPM_Project ${SOURCES})

# 手动链接 libi2c 库
target_link_libraries(BPM_Project PRIVATE i2c)
