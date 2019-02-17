cmake_minimum_required(VERSION 2.8.6)

project(osm_topological_planner)

########################################################################
# ROS dependency
########################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  genmsg 
  actionlib_msgs 
  actionlib
  std_msgs
  geometry_msgs
  osm_planner_msgs
  osm_trajectory_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime std_msgs actionlib_msgs geometry_msgs
)

