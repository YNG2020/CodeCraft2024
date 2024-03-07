cmake_minimum_required(VERSION 3.0)

project(MyProject)

# 添加头文件目录
include_directories(${CMAKE_SOURCE_DIR}/src)

# 声明一个可执行文件
add_executable(main ${CMAKE_SOURCE_DIR}/src/main.cpp)
