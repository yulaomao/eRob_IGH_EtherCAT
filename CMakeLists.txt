cmake_minimum_required(VERSION 3.10)
project(ethercat_erob)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)  # or Release, RelWithDebInfo, etc.

# EtherLab
set(ETHERLAB_DIR /usr/local/etherlab)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_library(ETHERCAT_LIB ethercat HINTS ${ETHERLAB_DIR}/lib)

include_directories(
  ${ETHERLAB_DIR}/include
  src/
)

# Find required packages
find_package(Threads REQUIRED)

# Original driver executable
add_executable(
  igh_driver
  src/igh_driver.cpp)
  
target_include_directories(
  igh_driver
  PRIVATE
  ${ETHERLAB_DIR}/include
)
target_link_libraries(igh_driver ${ETHERCAT_LIB})

# Motor controller library
add_library(motor_controller
  src/motor_controller.cpp
  src/motor_controller.h
)

target_include_directories(motor_controller
  PUBLIC
  ${ETHERLAB_DIR}/include
  src/
)

target_link_libraries(motor_controller 
  ${ETHERCAT_LIB}
  Threads::Threads
)

# Three-axis arm library
add_library(three_axis_arm
  src/three_axis_arm.cpp
  src/three_axis_arm.h
)

target_include_directories(three_axis_arm
  PUBLIC
  src/
)

target_link_libraries(three_axis_arm 
  motor_controller
)

# Arm demonstration executable
add_executable(
  arm_demo
  src/arm_demo.cpp
)

target_link_libraries(arm_demo 
  three_axis_arm
  motor_controller
)

# API test executable (can run without EtherCAT hardware)
add_executable(
  test_api
  src/test_api.cpp
)

target_link_libraries(test_api 
  three_axis_arm
  motor_controller
)








