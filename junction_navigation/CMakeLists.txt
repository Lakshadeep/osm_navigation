cmake_minimum_required(VERSION 2.8.3)
project(junction_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  gateway_detection
  robot_distance_monitor
  robot_heading_monitor
  gateway_msgs
  heading_control
  actionlib_msgs
  actionlib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES
  JunctionNavigation.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  gateway_msgs
  heading_control
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gateway_detection
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
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

add_library(junction_navigation src/junction_navigation.cpp)
target_link_libraries(junction_navigation ${catkin_LIBRARIES})

add_library(junction_navigation_ros src/junction_navigation_ros.cpp)
target_link_libraries(junction_navigation_ros junction_navigation ${catkin_LIBRARIES})
add_dependencies(junction_navigation_ros junction_navigation_generate_messages_cpp)

#add_executable(junction_navigation_node src/junction_navigation_node.cpp)
#target_link_libraries(junction_navigation_node junction_navigation_ros ${catkin_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${EIGEN3_INCLUDE_DIRS})

