cmake_minimum_required(VERSION 3.0)

project(MyProject)

# 添加头文件目录
include_directories(${CMAKE_SOURCE_DIR}/src)

# 声明一个可执行文件，并包含main.cpp、decisionmaker.cpp和global_vars.cpp
add_executable(main 
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    ${CMAKE_SOURCE_DIR}/src/decisionmaker.cpp
    ${CMAKE_SOURCE_DIR}/src/global_vars.cpp
)
