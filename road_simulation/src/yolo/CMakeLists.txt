project(yolo)
add_definitions(-DOPENCV)
include_directories(include)
# add_definitions(-DGPU)
# include_directories(/usr/local/cuda/include)
# add_definitions(-DCUDNN)
add_executable(${PROJECT_NAME} src/main.cpp src/YOLO_DARKNET.cpp)
add_library(${PROJECT_NAME}_lib SHARED src/main.cpp src/YOLO_DARKNET.cpp)

target_link_libraries(${PROJECT_NAME} road_proto ${PROTOBUF_LIBRARIES} ${OpenCV_LIBS} /usr/local/lib/libdarknet.so)
target_link_libraries(${PROJECT_NAME}_lib road_proto ${PROTOBUF_LIBRARIES} ${OpenCV_LIBS} /usr/local/lib/libdarknet.so)

