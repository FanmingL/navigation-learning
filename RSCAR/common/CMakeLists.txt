project(common)

# add_executable(${PROJECT_NAME} common_test.cpp)
add_library(rs SHARED rs.cpp image_util.cpp)
# target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})
