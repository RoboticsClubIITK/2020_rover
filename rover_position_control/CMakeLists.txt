cmake_minimum_required(VERSION 2.8.3)
project(rover_position_control)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(rover_position_control_lib src/rover_position_control.cpp)

cs_add_executable(rover_heading_node src/heading_node.cpp)
cs_add_executable(rover_position_control_node src/rover_position_control_node.cpp)

target_link_libraries(rover_position_control_node rover_position_control_lib)

cs_install()
cs_export()