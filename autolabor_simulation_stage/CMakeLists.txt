cmake_minimum_required(VERSION 2.8.3)
project(autolabor_simulation_stage)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf
  std_msgs
  message_generation
)

############
## Server ##
############
add_service_files(
  FILES
  Obstacle.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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

add_executable(simulation_stage_node src/simulation_stage_node.cpp)

add_dependencies(simulation_stage_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(simulation_stage_node
  ${catkin_LIBRARIES}
)
