project(separation)
protobuf_generate_cpp(PROTO_${PROJECT_NAME}_SRC PROTO_${PROJECT_NAME}_HDR proto/separation.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME} SHARED separation.cpp ${PROTO_${PROJECT_NAME}_SRC})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES})


