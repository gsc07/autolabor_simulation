cmake_minimum_required(VERSION 2.8.3)
project(location_fusion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    tf_conversions
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

add_executable(simple_fusion src/simple_fusion.cpp)

add_dependencies(simple_fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(simple_fusion
    ${catkin_LIBRARIES}
)
