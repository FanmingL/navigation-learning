project(track)
add_definitions(-DSUBPATH="${CMAKE_CURRENT_SOURCE_DIR}")

protobuf_generate_cpp(PROTO_${PROJECT_NAME}_SRC PROTO_${PROJECT_NAME}_HDR proto/track.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME}_proto SHARED ${PROTO_${PROJECT_NAME}_SRC})

target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

add_subdirectory(KFilter)

add_executable(${PROJECT_NAME} track.cpp ${PROTO_${PROJECT_NAME}_SRC})

add_dependencies(${PROJECT_NAME} KFilter)
add_dependencies(${PROJECT_NAME} detect_proto)

target_link_libraries(${PROJECT_NAME} rs KFilter detect_proto ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES})


