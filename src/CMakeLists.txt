cmake_minimum_required(VERSION 3.18.4)
project(src VERSION 0.1.0)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(src main.cpp
                   servo_func.cpp
                   trajectory_predictor.cpp
                   robot_arm.cpp
		   mapping.cpp)
target_link_libraries(src wiringPi)
target_link_libraries(src Eigen3::Eigen)
target_link_libraries(src ${OpenCV_LIBS})
target_link_libraries(src pthread)
