cmake_minimum_required(VERSION 3.0.2)
project(sign_light_driving)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_generation
)

add_message_files(
  FILES
  Detection.msg
  DetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs cv_bridge message_runtime
)

catkin_install_python(PROGRAMS
  scripts/detector_node.py
  scripts/controller_node.py
  scripts/collector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
