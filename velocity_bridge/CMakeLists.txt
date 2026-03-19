cmake_minimum_required(VERSION 3.8)
project(velocity_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  scripts/ros_velocity_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
