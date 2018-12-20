cmake_minimum_required(VERSION 2.8.6)

project(osm_map_server)


########################################################################
# ROS dependency
########################################################################
find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib
  std_msgs
  geometry_msgs
  osm_map_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


catkin_python_setup()


catkin_package(CATKIN_DEPENDS
  geometry_msgs osm_map_msgs
)

