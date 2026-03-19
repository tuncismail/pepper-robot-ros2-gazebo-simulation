cmake_minimum_required(VERSION 3.8)
project(pepper_laser_bridge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  scripts/pepper_laser_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
