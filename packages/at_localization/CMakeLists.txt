cmake_minimum_required(VERSION 2.8.3)
project(at_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  tf2_ros
  cv_bridge
  sensor_msgs
  geometry_msgs
  duckietown_msgs
)

catkin_package()
