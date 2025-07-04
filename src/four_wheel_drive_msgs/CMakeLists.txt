cmake_minimum_required(VERSION 3.0.2)
project(four_wheel_drive_msgs)

find_package(
  catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  four_wheel_drive_error_msg.msg
  four_wheel_drive_state_msg.msg
  four_wheel_drive_fb_lr_msg.msg
  four_wheel_drive_fb_msg.msg
  four_wheel_drive_full_msg.msg
  four_wheel_drive_base_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

#

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

#

#############
## Testing ##
#############

#
