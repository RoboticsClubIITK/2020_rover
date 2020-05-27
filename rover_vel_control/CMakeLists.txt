cmake_minimum_required(VERSION 2.8.3)
project(rover_vel_control)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(rover_vel_control_lib src/rover_vel.cpp)

cs_add_executable(rover_vel_node src/rover_vel_node.cpp)

target_link_libraries(rover_vel_node rover_vel_control_lib)

cs_install()
cs_export()