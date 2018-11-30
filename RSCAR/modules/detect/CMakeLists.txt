project(detect)
add_definitions(-DSUBPATH="${CMAKE_CURRENT_SOURCE_DIR}")

protobuf_generate_cpp(PROTO_${PROJECT_NAME}_SRC PROTO_${PROJECT_NAME}_HDR proto/detect.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(${PROJECT_NAME}_proto SHARED ${PROTO_${PROJECT_NAME}_SRC})

target_link_libraries(${PROJECT_NAME}_proto ${PROTOBUF_LIBRARIES})

add_subdirectory(BFSeparation)
add_subdirectory(Yolo)

add_executable(${PROJECT_NAME} detect.cpp ${PROTO_${PROJECT_NAME}_SRC})

add_dependencies(${PROJECT_NAME} BFSeparation)
add_dependencies(${PROJECT_NAME} Yolo)

target_link_libraries(${PROJECT_NAME} BFSeparation Yolo rs ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES})


