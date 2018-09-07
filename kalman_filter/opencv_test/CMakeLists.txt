find_package(OpenCV REQUIRED)
add_executable(main_opencv main.cpp)
add_dependencies(main_opencv kf)
target_link_libraries(main_opencv ${OpenCV_LIBS} kf)