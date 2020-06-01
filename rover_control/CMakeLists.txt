cmake_minimum_required(VERSION 3.0.2)
project(rover_control)

find_package(catkin_simple REQUIRED)

catkin_simple()
cs_add_executable(command_node
	src/command_node.cpp
	)
cs_install()
cs_export()
