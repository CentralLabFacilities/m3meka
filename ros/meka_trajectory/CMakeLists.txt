cmake_minimum_required(VERSION 2.8.3)
project(meka_trajectory)

## Find catkin macros and libraries
find_package(catkin REQUIRED 
  COMPONENTS
  actionlib_msgs
  trajectory_msgs
  rospy
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Traj.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)
