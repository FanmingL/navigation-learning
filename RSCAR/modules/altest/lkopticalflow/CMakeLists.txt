project(lkopticalflow)
protobuf_generate_cpp(PROTO_${PROJECT_NAME}_SRC PROTO_${PROJECT_NAME}_HDR proto/lkopticalflow.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME} SHARED lkopticalflow.cpp ${PROTO_${PROJECT_NAME}_SRC})

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES})


