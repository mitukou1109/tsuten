cmake_minimum_required(VERSION 3.0.2)
project(tsuten_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    dynamic_reconfigure
    geometry_msgs
    message_generation
    rospy
    std_msgs
)

add_action_files(FILES
  Perform.action
)

add_message_files(FILES
  Odometry.msg
  SensorStates.msg
  TapeLEDCommand.msg
)

add_service_files(FILES
  CorrectRobotPosition.srv
  ResetShooter.srv
  ShootBottle.srv  
  ShootOnTable.srv
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/BehaviorServer.cfg
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    dynamic_reconfigure
    geometry_msgs
    message_runtime
    rospy
    std_msgs
)