cmake_minimum_required(VERSION 3.0.2)
project(tsuten_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    dynamic_reconfigure
    message_generation
    std_msgs
)

add_action_files(FILES
  Perform.action
)

add_service_files(FILES
  ResetShooter.srv
  ShootBottle.srv  
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    dynamic_reconfigure
    message_runtime
    std_msgs
)