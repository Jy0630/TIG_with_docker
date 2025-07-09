cmake_minimum_required(VERSION 3.0.2)
project(object_detect)

## Find catkin macros and libraries
## message_generation 是產生 srv/msg 程式碼所必需的
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   BoundingBox.msg
#   BoundingBoxArray.msg
# )

add_service_files(
  FILES
  DetectOrangeGoal.srv
)

## Generate added messages and services with any dependencies listed here

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  LIBRARIES ball_detect
 CATKIN_DEPENDS rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include # 如果有C++ header files
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/coffee_detection.py
  scripts/coffee_supply.py
  scripts/orange_detection.py
  scripts/orange_detection_srv.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
