cmake_minimum_required(VERSION 2.8)
project(voxblox_planner)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_definitions(-std=c++17)

cs_add_library(${PROJECT_NAME}_lib
    src/graph_def.cpp
    src/point_sampler.cpp
    src/${PROJECT_NAME}.cpp
)
target_link_libraries(${PROJECT_NAME} ${BOOST_LIBRARIES} ${Eigen3_LIBRARIES})

cs_add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

cs_install()
cs_export()
