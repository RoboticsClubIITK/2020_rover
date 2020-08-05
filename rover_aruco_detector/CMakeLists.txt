cmake_minimum_required(VERSION 2.8.3)
project(rover_aruco_detector)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(aruco_marker_detect_lib src/aruco_marker_detect.cpp)
cs_add_library(pose_estimation_lib src/pose_estimation.cpp)

cs_add_executable(aruco_marker_detect_node src/aruco_marker_detect_node.cpp)
target_link_libraries(aruco_marker_detect_node aruco_marker_detect_lib ${OpenCV_LIBRARIES})

cs_add_executable(pose_estimation_node src/pose_estimation_node.cpp)
target_link_libraries(pose_estimation_node pose_estimation_lib)

cs_install()
cs_export()
