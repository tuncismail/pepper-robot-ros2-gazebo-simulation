cmake_minimum_required(VERSION 3.8)
project(gazebo_model_velocity_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(gazebo_ros_model_velocity SHARED
  src/gazebo_ros_model_velocity.cpp
  src/speed_limiter.cpp
)

ament_target_dependencies(gazebo_ros_model_velocity
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  gazebo_ros
  gazebo_dev
)

target_link_libraries(gazebo_ros_model_velocity
  ${GAZEBO_LIBRARIES}
)

install(TARGETS gazebo_ros_model_velocity
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
