add_executable(main_test main.cpp)
add_dependencies(main_test kf)
target_link_libraries(main_test kf)