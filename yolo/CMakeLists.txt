cmake_minimum_required(VERSION 3.0)
project(yolo)
set(CMAKE_CXX_STANDARD 14)
add_definitions(-DPATH="${CMAKE_CURRENT_SOURCE_DIR}")
find_package(OpenCV REQUIRED)
include_directories(src/ /usr/local/include/)
add_executable(main src/main.cpp src/YOLO_DARKNET.cpp src/YOLO_DARKNET.h)
target_link_libraries(main ${OpenCV_LIBS} /usr/local/lib/libdarknet.a)

