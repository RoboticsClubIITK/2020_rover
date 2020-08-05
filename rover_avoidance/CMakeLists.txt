cmake_minimum_required(VERSION 3.0.2)
project(rover_avoidance)

find_package(catkin_simple)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_executable(lidar_control src/avoidance.cpp)
target_link_libraries(lidar_control ${catkin_LIBRARIES})

cs_install()
cs_export()
