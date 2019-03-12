cmake_minimum_required(VERSION 2.8.3)
project(room_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  gateway_detection
  robot_distance_monitor
  robot_heading_monitor
  gateway_msgs
  osm_map_msgs
  heading_control
  actionlib_msgs
  actionlib
  motion_control
)

add_action_files(
  DIRECTORY action
  FILES
  RoomNavigation.action
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  gateway_msgs
  osm_map_msgs
  heading_control
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime std_msgs actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(room_navigation src/room_navigation.cpp)
target_link_libraries(room_navigation ${catkin_LIBRARIES})

add_library(room_navigation_ros src/room_navigation_ros.cpp)
target_link_libraries(room_navigation_ros room_navigation ${catkin_LIBRARIES})
add_dependencies(room_navigation_ros room_navigation_generate_messages_cpp)

add_executable(room_navigation_node src/room_navigation_node.cpp)
target_link_libraries(room_navigation_node room_navigation_ros ${catkin_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${EIGEN3_INCLUDE_DIRS})

