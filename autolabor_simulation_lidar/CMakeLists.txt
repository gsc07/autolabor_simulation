cmake_minimum_required(VERSION 2.8.3)
project(autolabor_simulation_lidar)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simulation_lidar_node src/simulation_lidar_node.cpp)

add_dependencies(simulation_lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(simulation_lidar_node
  ${catkin_LIBRARIES}
)

