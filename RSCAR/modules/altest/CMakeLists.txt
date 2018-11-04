project(altest)
add_definitions(-DSUBPATH="${CMAKE_CURRENT_SOURCE_DIR}")

protobuf_generate_cpp(PROTO_${PROJECT_NAME}_SRC PROTO_${PROJECT_NAME}_HDR proto/altest.proto)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_subdirectory(lkopticalflow)
add_subdirectory(separation)
add_subdirectory(static_stabling)

add_executable(${PROJECT_NAME} altest.cpp ${PROTO_${PROJECT_NAME}_SRC})

add_dependencies(${PROJECT_NAME} lkopticalflow separation)

target_link_libraries(${PROJECT_NAME} rs lkopticalflow separation static_stabling ${OpenCV_LIBS} ${PROTOBUF_LIBRARIES})


