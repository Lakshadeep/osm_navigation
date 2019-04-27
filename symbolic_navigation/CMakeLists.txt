cmake_minimum_required(VERSION 2.8.3)
project(symbolic_navigation)

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
  corridor_navigation
  area_navigation
  door_passing
  junction_maneuvering
  semantic_localization
  osm_topological_planner
  actionlib_msgs
  actionlib
  osm_planner_msgs
)

add_action_files(
  DIRECTORY action
  FILES
  SymbolicNavigation.action
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  osm_map_msgs
  actionlib_msgs
  osm_planner_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime std_msgs actionlib_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  osm_map_msgs
  actionlib_msgs
  osm_planner_msgs
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

add_library(symbolic_navigation src/symbolic_navigation.cpp)
target_link_libraries(symbolic_navigation ${catkin_LIBRARIES})

add_library(symbolic_navigation_ros src/symbolic_navigation_ros.cpp)
target_link_libraries(symbolic_navigation_ros symbolic_navigation ${catkin_LIBRARIES} ${LIBS})
add_dependencies(symbolic_navigation_ros symbolic_navigation_generate_messages_cpp)

add_executable(symbolic_navigation_node src/symbolic_navigation_node.cpp)
target_link_libraries(symbolic_navigation_node symbolic_navigation_ros ${catkin_LIBRARIES} ${LIBS})

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${EIGEN3_INCLUDE_DIRS})

