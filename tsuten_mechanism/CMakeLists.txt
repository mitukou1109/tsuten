cmake_minimum_required(VERSION 3.0.2)
project(tsuten_mechanism)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    tsuten_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    tsuten_msgs
  INCLUDE_DIRS include
  LIBRARIES solenoid_valve_controller bottle_shooter_controller tape_led_controller
)

add_library(solenoid_valve_controller
  src/solenoid_valve_controller.cpp
)
target_include_directories(solenoid_valve_controller
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(solenoid_valve_controller
  ${catkin_LIBRARIES}
)

add_library(bottle_shooter_controller
  src/bottle_shooter_controller.cpp
)
target_include_directories(bottle_shooter_controller
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(bottle_shooter_controller
  solenoid_valve_controller
  ${catkin_LIBRARIES}
)

add_executable(bottle_shooter_controller_node
  src/bottle_shooter_controller_node.cpp
)
target_include_directories(bottle_shooter_controller_node
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(bottle_shooter_controller_node
  bottle_shooter_controller
  ${catkin_LIBRARIES}
)

add_library(tape_led_controller
  src/tape_led_controller.cpp
)
target_include_directories(tape_led_controller
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(tape_led_controller
  ${catkin_LIBRARIES}
)

install(TARGETS solenoid_valve_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS bottle_shooter_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS tape_led_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)