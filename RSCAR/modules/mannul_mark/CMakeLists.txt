project(mannul_mark)
add_definitions(-DSUBPATH="${CMAKE_CURRENT_SOURCE_DIR}")

protobuf_generate_cpp(PROTO_${PROJECT_NAME}_SRC PROTO_${PROJECT_NAME}_HDR proto/mannul_mark.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME}_proto SHARED ${PROTO_${PROJECT_NAME}_SRC})

target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

add_executable(${PROJECT_NAME} mannul_mark.cpp ${PROTO_${PROJECT_NAME}_SRC})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_proto)

target_link_libraries(${PROJECT_NAME} detect_proto ${PROJECT_NAME}_proto rs ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES})


add_executable(${PROJECT_NAME}_test mannul_test.cpp ${PROTO_${PROJECT_NAME}_SRC})

add_dependencies(${PROJECT_NAME}_test ${PROJECT_NAME}_proto)

target_link_libraries(${PROJECT_NAME}_test detect_proto ${PROJECT_NAME}_proto rs ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES})
