cmake_minimum_required(VERSION 2.8.3)
project(rover_position_control)

set (CMAKE_MODULE_PATH ~/rover_summer_project/src/GeographicLib-1.50.1/cmake)

find_package(catkin_simple REQUIRED)
find_package (GeographicLib REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(rover_position_control_lib src/rover_position_control.cpp)
cs_add_library(rover_gps_lib src/rover_gps_world.cpp)

cs_add_executable(rover_position_control_node src/rover_position_control_node.cpp)
cs_add_executable(rover_heading_node src/heading_node.cpp)
cs_add_executable(rover_gps_node src/rover_gps_world_node.cpp)

target_link_libraries(rover_position_control_node rover_position_control_lib)
target_link_libraries(rover_gps_node rover_gps_lib)
target_link_libraries (rover_gps_node ${GeographicLib_LIBRARIES})

cs_install()
cs_export()