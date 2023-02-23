cmake_minimum_required(VERSION 3.0.2)
project(tsuten_navigation)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
    tf2
    tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    std_msgs
    tf2
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(four_omni_drive_controller
  src/four_omni_drive_controller.cpp
)
target_link_libraries(four_omni_drive_controller
  ${catkin_LIBRARIES}
)

add_executable(odom_tf_publisher
  src/odom_tf_publisher.cpp
)
target_link_libraries(odom_tf_publisher
  ${catkin_LIBRARIES}
)