cmake_minimum_required(VERSION 2.8.3)
project(rover_obstacle_avoidance)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(rover_obstacle_avoid_lib src/obstacle_avoid.cpp)

cs_add_executable(rover_obstacle_avoidance_node src/obstacle_avoid_node.cpp)

target_link_libraries(rover_obstacle_avoidance_node rover_obstacle_avoid_lib)

cs_install()
cs_export()