project(replay)

protobuf_generate_cpp(PROTO_${PROJECT_NAME}_SRC PROTO_${PROJECT_NAME}_HDR proto/replay.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_package(Torch REQUIRED)

include_directories(${TORCH_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} replay.cpp ${PROTO_${PROJECT_NAME}_SRC})

add_dependencies(${PROJECT_NAME} relocate_proto information_proto)

target_link_libraries(${PROJECT_NAME} "${TORCH_LIBRARIES}" relocate_proto information_proto rs ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES})


