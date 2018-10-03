project(stabling)
include_directories(include ../car_filter/include)
add_executable(${PROJECT_NAME} src/main.cpp src/stabling.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} car_filter_lib)
