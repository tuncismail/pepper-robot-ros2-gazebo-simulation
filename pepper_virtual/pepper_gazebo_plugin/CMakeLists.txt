cmake_minimum_required(VERSION 3.8)
project(pepper_gazebo_plugin)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
  scripts/laser_publisher.py
  scripts/odom_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch worlds models config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
