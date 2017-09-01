cmake_minimum_required(VERSION 2.8.3)
project(west_tools)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  PackList.srv
  NodeList.srv
  ServiceList.srv
  RunNode.srv
  KillNode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## catkin specific configuration
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES west_tools
#  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Build
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#${catkin_EXPORTED_TARGETS})
