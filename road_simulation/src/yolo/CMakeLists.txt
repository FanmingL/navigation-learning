project(yolo)
add_definitions(-DOPENCV)
include_directories(include)
# add_definitions(-DGPU)
# include_directories(/usr/local/cuda/include)
# add_definitions(-DCUDNN)
add_executable(${PROJECT_NAME} src/main.cpp src/YOLO_DARKNET.cpp )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} /usr/local/lib/libdarknet.a)

