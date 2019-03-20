cmake_minimum_required(VERSION 2.8.3)
project(area_navigation)

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
  robot_distance_monitor
  robot_heading_monitor
  navigation_sign_msgs
  osm_map_msgs
  heading_control
  actionlib_msgs
  actionlib
  motion_control
  navigation_sign_detection
)

add_action_files(
  DIRECTORY action
  FILES
  AreaNavigation.action
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  navigation_sign_msgs
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

add_library(area_navigation src/area_navigation.cpp)
target_link_libraries(area_navigation ${catkin_LIBRARIES})

add_library(area_navigation_ros src/area_navigation_ros.cpp)
target_link_libraries(area_navigation_ros area_navigation ${catkin_LIBRARIES})
add_dependencies(area_navigation_ros area_navigation_generate_messages_cpp)

add_executable(area_navigation_node src/area_navigation_node.cpp)
target_link_libraries(area_navigation_node area_navigation_ros ${catkin_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${EIGEN3_INCLUDE_DIRS})

