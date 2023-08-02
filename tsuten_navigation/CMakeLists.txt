cmake_minimum_required(VERSION 3.0.2)
project(tsuten_navigation)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    geometry_msgs
    move_base_msgs
    nav_msgs
    roscpp
    tf2
    tf2_ros
    tsuten_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    move_base_msgs
    nav_msgs
    roscpp
    tf2
    tf2_ros
    tsuten_msgs
  INCLUDE_DIRS include
  LIBRARIES navigation_handler
)

add_library(navigation_handler
  src/navigation_handler.cpp
)
target_include_directories(navigation_handler
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(navigation_handler
  ${catkin_LIBRARIES}
)

add_executable(localization_helper
  src/localization_helper.cpp
)
target_include_directories(localization_helper
  PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(localization_helper
  ${catkin_LIBRARIES}
)

install(TARGETS navigation_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)