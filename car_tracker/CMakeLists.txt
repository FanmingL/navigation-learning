cmake_minimum_required(VERSION 3.0)
project(car_tracker)
set(CMAKE_CXX_STANDARD 14)
add_definitions(-DPATH="${CMAKE_CURRENT_SOURCE_DIR}")
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
add_subdirectory(kalman_filter)
include_directories(${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        /usr/include
        ${CMAKE_CURRENT_SOURCE_DIR}/kalman_filter)
set(SRC src/main.cpp src/car_tracker.cpp)
add_executable(main ${SRC})
target_link_libraries(main ${OpenCV_LIBS} kf)
