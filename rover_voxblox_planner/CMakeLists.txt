cmake_minimum_required(VERSION 2.8)
project(rover_voxblox_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE=Release)

cs_add_library(${PROJECT_NAME}_lib src/point_sampler.cpp src/voxblox_planner.cpp src/publish_waypoint.cpp)
target_link_libraries(${PROJECT_NAME}_lib ${BOOST_LIBRARIES} ${Eigen3_LIBRARIES})

cs_add_executable(${PROJECT_NAME}_node src/voxblox_planner_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

cs_add_executable(publish_waypoint_node src/publish_waypoint_node.cpp)
target_link_libraries(publish_waypoint_node ${PROJECT_NAME}_lib)

cs_install()
cs_export()
