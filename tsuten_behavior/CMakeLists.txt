cmake_minimum_required(VERSION 3.0.2)
project(tsuten_behavior)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    jsk_rviz_plugins
    pcl_conversions
    pcl_ros
    pluginlib
    roscpp
    rviz
    std_msgs
    tf2
    tf2_ros
    tsuten_mechanism
    tsuten_msgs
    tsuten_navigation
    visualization_msgs
)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED
  Core
  Widgets  
)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

find_package(PCL 1.10 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    jsk_rviz_plugins
    pcl_conversions
    pcl_ros
    pluginlib
    roscpp
    rviz
    std_msgs
    tf2
    tf2_ros
    tsuten_mechanism
    tsuten_msgs
    tsuten_navigation
    visualization_msgs
  INCLUDE_DIRS include
  LIBRARIES behavior_control_panel
)

add_library(behavior_control_panel
  include/tsuten_behavior/behavior_control_panel.h
  src/behavior_control_panel.cpp
  src/behavior_control_panel_ui.cpp
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

add_executable(behavior_server
  src/behavior_server.cpp
)
add_dependencies(behavior_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(behavior_server
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(behavior_server
  ${catkin_LIBRARIES}
)

add_executable(behavior_visualizer
  src/behavior_visualizer.cpp
)
target_include_directories(behavior_visualizer
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(behavior_visualizer
  ${catkin_LIBRARIES}
)

add_executable(table_manager
  src/table_manager.cpp
)
target_include_directories(table_manager
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(table_manager
  ${catkin_LIBRARIES}
)
target_compile_features(table_manager PUBLIC cxx_std_17)