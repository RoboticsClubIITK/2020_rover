cmake_minimum_required(VERSION 2.8.3)
project(rover_box_detector)

find_package(catkin_simple)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

cs_add_executable(${PROJECT_NAME}_node src/box_detect_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})

cs_install()
cs_export()
