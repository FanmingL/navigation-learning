project(static_stabling)
protobuf_generate_cpp(PROTO_${PROJECT_NAME}_SRC PROTO_${PROJECT_NAME}_HDR proto/static_stabling.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME} SHARED static_stabling.cpp ${PROTO_${PROJECT_NAME}_SRC})

target_link_libraries(${PROJECT_NAME} detect_proto ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES})


