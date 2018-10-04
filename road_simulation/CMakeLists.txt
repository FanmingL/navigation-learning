cmake_minimum_required(VERSION 3.0)
project(road_simulation)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH  ${PROJECT_BINARY_DIR}/lib)

add_definitions(-DPATH="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DDIMENSION=2)
add_definitions(-DSHOW_VIDEO)
add_definitions(-DWRITE_VIDEO)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} 
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        /usr/include
        /usr/lcoal/include)
add_subdirectory(src)


