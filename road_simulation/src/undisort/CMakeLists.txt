project(undisort)
include_directories(include)
add_executable(${PROJECT_NAME} src/main.cpp src/undisort.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})