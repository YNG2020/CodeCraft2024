cmake_minimum_required(VERSION 3.0)

project(MyProject)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# 添加头文件目录
include_directories(${CMAKE_SOURCE_DIR}/src)

# 声明一个可执行文件，并包含main.cpp、decisionmaker.cpp和global_vars.cpp
add_executable(main 
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    ${CMAKE_SOURCE_DIR}/src/decision_maker_base.cpp
    ${CMAKE_SOURCE_DIR}/src/decision_maker_robot.cpp
    ${CMAKE_SOURCE_DIR}/src/decision_maker_ship.cpp
    ${CMAKE_SOURCE_DIR}/src/decision_maker_robot_jamControl.cpp
    ${CMAKE_SOURCE_DIR}/src/global_vars.cpp
)
