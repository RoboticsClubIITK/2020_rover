cmake_minimum_required(VERSION 3.0.2)
project(pose_control)

find_package(catkin_simple REQUIRED)

catkin_simple()
cs_add_executable(pose_navigation_node
		src/pose_navigation_node.cpp
		)
cs_install()
cs_export()
