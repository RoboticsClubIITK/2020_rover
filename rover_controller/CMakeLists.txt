cmake_minimum_required(VERSION 3.0.2)
project(rover_controller)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MODULE_PATH /home/dnyandeep/GeographicLib-1.50.1/cmake)
add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(GeographicLib REQUIRED)


cs_add_executable(command_node src/command_node.cpp)
cs_add_executable(pose_navigation_node src/pose_navigation_node.cpp)
cs_add_executable(gps_node src/gps_node.cpp)

target_link_libraries (gps_node ${GeographicLib_LIBRARIES})

cs_install()
cs_export()
