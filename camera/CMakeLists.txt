project(camera_test)
cmake_minimum_required(VERSION 2.8.3)
find_package(OpenCV 4 REQUIRED)
include_directories("/usr/include/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS})
add_library(CAM STATIC
            include/camera.h
            src/camera.cpp)


add_executable(test src/test.cpp)
target_link_libraries(test CAM yaml-cpp ${OpenCV_LIBS})