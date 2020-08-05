cmake_minimum_required(VERSION 3.0.2)
project(rover_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs   
    nav_msgs
    sensor_msgs
)
find_package(GeographicLib REQUIRED)
catkin_package(
)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GeographicLib_INCLUDE_DIRS}
)
add_executable(target src/target.cpp)
target_link_libraries(target  ${GeographicLib_LIBRARIES} ${catkin_LIBRARIES})
add_executable(control src/control.cpp)
target_link_libraries(control ${catkin_LIBRARIES})
add_executable(velocity src/velocity.cpp)
target_link_libraries(velocity  ${catkin_LIBRARIES})