cmake_minimum_required(VERSION 3.8)
project(tsuten_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

set(CMAKE_AUTOMOC ON)

ament_auto_add_library(behavior_control_panel SHARED
  include/tsuten_behavior/behavior_control_panel.h
  src/behavior_control_panel.cpp
  src/behavior_control_panel_ui.cpp
)

ament_auto_add_executable(behavior_server
  src/behavior_server.cpp
)

ament_auto_add_executable(behavior_visualizer
  src/behavior_visualizer.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(DIRECTORY images launch rviz DESTINATION share/${PROJECT_NAME})

ament_auto_package()