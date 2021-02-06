cmake_minimum_required(VERSION 3.16)
find_package(Eigen3 REQUIRED)

add_library(utils INTERFACE )
target_sources(utils INTERFACE ${CMAKE_CURRENT_LIST_DIR}/src/utility_functions.h)

add_executable(test_pf_exe src/test_pf.cpp  src/KFAdapter.cpp 
                                            src/EKFAdapter.cpp
                                            src/UKFAdapter.cpp  
                                            src/ParticleFilter.cpp                                       
                                            ${CMAKE_CURRENT_LIST_DIR}/../motion_models/src/ConstantVelocityModel.cpp 
                                            ${CMAKE_CURRENT_LIST_DIR}/../motion_models/src/CoordinatedTurnModel.cpp
                                            ${CMAKE_CURRENT_LIST_DIR}/../measurement_models/src/CvMeasurementModel.cpp 
                                            ${CMAKE_CURRENT_LIST_DIR}/../measurement_models/src/DualBearingMeasModel.cpp)

target_include_directories ( test_pf_exe PUBLIC 
                             ${CMAKE_CURRENT_LIST_DIR}/src/
                             ${CMAKE_CURRENT_LIST_DIR}/../motion_models/src/
                             ${CMAKE_CURRENT_LIST_DIR}/../measurement_models/src/
                            )
target_link_libraries(test_pf_exe INTERFACE utils PUBLIC Eigen3::Eigen)