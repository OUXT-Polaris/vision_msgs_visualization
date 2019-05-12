cmake_minimum_required(VERSION 2.8.3)
project(vision_msgs_visualization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  sensor_msgs
  vision_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES vision_msgs_visualization
  CATKIN_DEPENDS message_filters roscpp sensor_msgs vision_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
