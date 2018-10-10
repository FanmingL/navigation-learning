project(tracker)
add_definitions(-DSUBPATH="${CMAKE_CURRENT_SOURCE_DIR}")

protobuf_generate_cpp(PROTO_${PROJECT_NAME}_SRC PROTO_${PROJECT_NAME}_HDR proto/tracker.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(${PROJECT_NAME} src/tracker.cpp ${PROTO_${PROJECT_NAME}_SRC})

target_link_libraries(${PROJECT_NAME} rs ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES})


