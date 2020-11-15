cmake_minimum_required(VERSION 3.0.2)
project(lane_detection)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
