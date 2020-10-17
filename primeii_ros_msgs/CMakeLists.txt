cmake_minimum_required(VERSION 3.0.2)
project(primeii_ros_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  geometry_msgs
  std_msgs
)

add_message_files(
  FILES
  FingerFlex.msg
  GloveData.msg
  GlovesData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES primeii_ros_msgs
#  CATKIN_DEPENDS geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
