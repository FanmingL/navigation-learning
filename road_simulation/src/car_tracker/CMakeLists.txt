project(car_tracker)
# add_definitions(-DWRITE_VIDEO)
include_directories(${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        /usr/include
        include )
set(SRC src/main.cpp src/car_tracker.cpp src/KalmanFilter.cpp)
add_executable(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
