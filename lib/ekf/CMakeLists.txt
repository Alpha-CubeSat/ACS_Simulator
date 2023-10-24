cmake_minimum_required(VERSION 3.12)
project(EKF)

set(CMAKE_CXX_STANDARD 17)

# Find the Eigen library
find_package(Eigen3 REQUIRED)

# Add the EKF source and header files
add_library(EKF ekf.cpp ekf.h)

# Include Eigen headers
target_include_directories(EKF PRIVATE ${EIGEN3_INCLUDE_DIR})

# I haven't a clue how to hook this up to the rest of the software.
# add_executable(main main.cpp)
# target_link_libraries(main EKF)
