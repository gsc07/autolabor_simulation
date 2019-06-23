cmake_minimum_required(VERSION 2.8.3)
project(path_rviz_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  path_server
  random_numbers
  rviz
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES path_rviz_plugin
#  CATKIN_DEPENDS roscpp rviz
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})


set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
    message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
    ## pull in all required include dirs, define QT_LIBRARIES, etc.
    include(${QT_USE_FILE})
else()
    message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
    find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
    ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
    set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)

add_library(path_rviz_plugin src/path_rviz_plugin.cpp include/path_rviz_plugin/path_rviz_plugin.h)
add_dependencies(path_rviz_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_rviz_plugin ${QT_LIBRARIES} ${catkin_LIBRARIES})
