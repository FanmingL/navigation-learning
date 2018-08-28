cmake_minimum_required(VERSION 2.8.3)
project(cpp_map)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  std_msgs
  tf
)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/main.cpp src/PID.cpp src/CarControl.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
