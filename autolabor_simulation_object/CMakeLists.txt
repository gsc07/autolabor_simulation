cmake_minimum_required(VERSION 2.8.3)
project(autolabor_simulation_object)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autolabor_simulation_stage
  visualization_msgs
  interactive_markers
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

add_executable(simulation_object_node src/simulation_object_node.cpp)

add_dependencies(simulation_object_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(simulation_object_node
  ${catkin_LIBRARIES}
)

