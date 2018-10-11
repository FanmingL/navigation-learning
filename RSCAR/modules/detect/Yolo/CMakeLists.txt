project(Yolo)

protobuf_generate_cpp(PROTO_${PROJECT_NAME}_SRC PROTO_${PROJECT_NAME}_HDR proto/YoloConfig.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME} SHARED YoloDetection.cpp ${PROTO_${PROJECT_NAME}_SRC})

add_dependencies(${PROJECT_NAME} detect_proto)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES} detect_proto /usr/local/lib/libdarknet.a)
