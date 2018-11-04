project(BFSeparation)

protobuf_generate_cpp(PROTO_${PROJECT_NAME}_SRC PROTO_${PROJECT_NAME}_HDR proto/BFSeparation.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(BFSeparation SHARED BFSeparation.cpp ${PROTO_${PROJECT_NAME}_SRC})

add_dependencies(BFSeparation detect_proto)

target_link_libraries(BFSeparation rs ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES} detect_proto)
