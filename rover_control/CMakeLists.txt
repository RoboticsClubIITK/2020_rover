cmake_minimum_required(VERSION 3.0.2)
project(rover_control)

find_package(catkin_simple)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})

cs_add_executable(target src/target.cpp)
target_link_libraries(target  ${GeographicLib_LIBRARIES} ${catkin_LIBRARIES})

cs_add_executable(control src/control.cpp)
target_link_libraries(control ${catkin_LIBRARIES})

cs_add_executable(velocity src/velocity.cpp)
target_link_libraries(velocity  ${catkin_LIBRARIES})
