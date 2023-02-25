cmake_minimum_required(VERSION 3.0.2)
project(tsuten_behavior)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    roscpp
    rviz
    tsuten_mechanism
    tsuten_msgs
)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets  
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    roscpp
    rviz
    tsuten_mechanism
    tsuten_msgs
  INCLUDE_DIRS include
  LIBRARIES behavior_control_panel
)

add_executable(behavior_server
  src/behavior_server.cpp
)
add_dependencies(behavior_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(behavior_server
  PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(behavior_server
  ${catkin_LIBRARIES}
)

add_library(behavior_control_panel
  include/tsuten_behavior/behavior_control_panel.h
  src/behavior_control_panel.cpp
)
target_include_directories(behavior_control_panel
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(behavior_control_panel
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
)