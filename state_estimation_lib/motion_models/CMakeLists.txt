cmake_minimum_required(VERSION 3.16)
find_package(Eigen3 REQUIRED)

add_executable(test_motion_model test_main.cpp ConstantVelocityModel.cpp CoordinatedTurnModel.cpp)

#target_include_directories(test_motion_model PUBLIC "${PROJECT_SOURCE_DIR"} ConstantVelocityModel.h"

target_link_libraries(test_motion_model PRIVATE Eigen3::Eigen)
