cmake_minimum_required(VERSION 3.0.2)
project(owl_moveit_driver)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  rospy
  sensor_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})