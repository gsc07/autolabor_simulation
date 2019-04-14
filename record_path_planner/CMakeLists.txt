cmake_minimum_required(VERSION 2.8.3)
project(record_path_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    costmap_2d
    nav_core
    pluginlib
    roscpp
    )

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(record_path_planner src/record_path_planner.cpp)

#add_executable(record_path_planner_node src/record_path_planner.cpp)
#
#add_dependencies(record_path_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
target_link_libraries(record_path_planner
    ${catkin_LIBRARIES}
    )
