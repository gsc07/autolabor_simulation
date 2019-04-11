cmake_minimum_required(VERSION 2.8.3)
project(autolabor_simulation_location)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    geometry_msgs
    )


catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(simulation_location_node src/simulation_location_node.cpp)

add_dependencies(simulation_location_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(simulation_location_node
    ${catkin_LIBRARIES}
    )
