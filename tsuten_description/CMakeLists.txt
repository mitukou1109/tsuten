cmake_minimum_required(VERSION 3.8)
project(tsuten_description)

find_package(ament_cmake_auto REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()