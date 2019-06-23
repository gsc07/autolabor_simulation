cmake_minimum_required(VERSION 2.8.3)
project(path_server)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    roslib
    message_generation
    )

## Generate services in the 'srv' folder
add_service_files(
    FILES
    SetPathName.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_executable(record_path_node src/record_path_node.cpp)
add_dependencies(record_path_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_path_node
    ${catkin_LIBRARIES}
    )

add_executable(load_path_node src/load_path_node.cpp)
add_dependencies(load_path_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(load_path_node
    ${catkin_LIBRARIES}
    )
