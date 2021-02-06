cmake_minimum_required(VERSION 3.0)
project (sel)

find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE SE_SRC_FILES 
     "${CMAKE_CURRENT_LIST_DIR}/motion_models/src/*.cpp"
     "${CMAKE_CURRENT_LIST_DIR}/measurement_models/src/*.cpp"
     "${CMAKE_CURRENT_LIST_DIR}/filters/src/*.cpp"
     )

file(GLOB_RECURSE SE_INC_FILES 
     "${CMAKE_CURRENT_LIST_DIR}/motion_models/src/*.h"
     "${CMAKE_CURRENT_LIST_DIR}/measurement_models/src/*.h"
     "${CMAKE_CURRENT_LIST_DIR}/filters/src/*.h"
     )
add_library(se_lib ${SE_SRC_FILES} ${SE_INC_FILES})

target_include_directories ( se_lib PUBLIC 
                             ${CMAKE_CURRENT_LIST_DIR}/motion_models/src/
                             ${CMAKE_CURRENT_LIST_DIR}/measurement_models/src/
                             ${CMAKE_CURRENT_LIST_DIR}/filters/src/
                            )
target_link_libraries(se_lib Eigen3::Eigen)
