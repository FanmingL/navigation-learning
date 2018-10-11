project(common)

# add_executable(${PROJECT_NAME} common_test.cpp)
add_library(rs SHARED rs.cpp)
# target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})
