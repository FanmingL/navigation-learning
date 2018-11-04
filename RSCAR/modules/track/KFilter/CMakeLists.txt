project(KFilter)

protobuf_generate_cpp(PROTO_${PROJECT_NAME}_SRC PROTO_${PROJECT_NAME}_HDR proto/KFilter.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME} SHARED KFilter.cpp ${PROTO_${PROJECT_NAME}_SRC} KalmanFilter.cpp)

target_link_libraries(${PROJECT_NAME} rs ${PROTOBUF_LIBRARIES} ${OpenCV_LIBS})
