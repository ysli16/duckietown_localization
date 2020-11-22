cmake_minimum_required(VERSION 2.8.3)
project(encoder_localization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  duckietown_msgs
  geometry_msgs
  message_generation
)


add_service_files(
  FILES
  FrameCalibration.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  duckietown_msgs
)

catkin_package(
 CATKIN_DEPENDS 
 message_runtime
)
 
