cmake_minimum_required(VERSION 2.8.3)
project(robot_control)
set(client ${PROJECT_NAME}_client)
set(server ${PROJECT_NAME}_server)
set(global ${PROJECT_NAME}_global)
set(local ${PROJECT_NAME}_local)
set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)
add_action_files(
  FILES
  GlobalPlanner.action
  SetTarget.action
)
generate_messages(
  DEPENDENCIES
  actionlib_msgs  
  geometry_msgs
  nav_msgs
  std_msgs
)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  /home/erdou/workspace/devel/include
  src/util
  src/server
  src/client
  src/global_planner
  src/local_planner
)

protobuf_generate_cpp(PROTO_PID_SRC PROTO_PID_HDR proto/pid.proto)

add_executable(${client} src/client/client.cpp src/util/PID.cpp)

add_executable(${server} src/server/server.cpp src/util/PID.cpp ${PROTO_PID_SRC})

add_executable(${global} src/global_planner/global_planner.cpp src/util/a_star.cpp)

add_executable(${local} src/local_planner/local_planner.cpp)

add_dependencies(${client} ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(${server} ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(${global} ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(${global} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(${client}
  ${catkin_LIBRARIES} 
  ${PROTOBUF_LIBRARIES}
)

target_link_libraries(${server}
  ${catkin_LIBRARIES} 
  ${PROTOBUF_LIBRARIES}
)













