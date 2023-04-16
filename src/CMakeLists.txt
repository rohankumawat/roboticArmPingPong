cmake_minimum_required(VERSION 3.0.0)
project(rago_cmake VERSION 0.1.0)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(rago_cmake main.cpp
                robot_arm.cpp
)
target_link_libraries( main ${OpenCV_LIBS} )

