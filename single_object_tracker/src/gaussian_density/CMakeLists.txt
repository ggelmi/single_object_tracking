cmake_minimum_required(VERSION 3.16)
find_package(Eigen3 REQUIRED)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../../state_estimation_lib se_build)

add_executable(gauss_density_exe  src/test_gauss_density.cpp
                                  src/GaussianDensity.cpp
)

target_include_directories ( gauss_density_exe PUBLIC
                             ${CMAKE_CURRENT_LIST_DIR}/src/

                            )
target_link_libraries(gauss_density_exe se_lib)