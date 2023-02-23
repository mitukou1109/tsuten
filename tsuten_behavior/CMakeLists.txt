cmake_minimum_required(VERSION 3.0.2)
project(tsuten_behavior)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    roscpp
    tsuten_mechanism
    tsuten_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    roscpp
    tsuten_mechanism
    tsuten_msgs
)

add_executable(behavior_server
  src/behavior_server.cpp
)
add_dependencies(behavior_server
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_include_directories(behavior_server
  PUBLIC ${catkin_INCLUDE_DIRS}
)
target_link_libraries(behavior_server
  ${catkin_LIBRARIES}
)