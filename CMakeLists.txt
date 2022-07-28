cmake_minimum_required(VERSION 3.0.2)
project(move_to_marker)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aruco
  rospy
  std_msgs
  nav_msgs
  tf
  tf_conversions
  message_generation
)


generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS aruco rospy std_msgs tf nav_msgs message_runtime
  DEPENDS system_lib
)

