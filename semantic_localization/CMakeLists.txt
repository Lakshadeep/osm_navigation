cmake_minimum_required(VERSION 2.8.3)
project(semantic_localization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_srvs
    tf
    nav_msgs
    std_srvs
    osm_map_msgs
)

find_package(Boost REQUIRED)

catkin_package(
    CATKIN_DEPENDS roscpp tf nav_msgs std_srvs osm_map_msgs
    INCLUDE_DIRS include
    LIBRARIES semantic_localization_map semantic_localization_pf semantic_localization_sensors
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(semantic_localization_pf src/semantic_localization/pf/pf.cpp src/semantic_localization/pf/pf_kdtree.c src/semantic_localization/pf/pf_pdf.c src/semantic_localization/pf/pf_vector.c src/semantic_localization/pf/eig3.c)

add_library(semantic_localization_map src/semantic_localization/map/semantic_map.c)

add_library(semantic_localization_sensors src/semantic_localization/sensors/sensor.cpp src/semantic_localization/sensors/odom.cpp src/semantic_localization/sensors/wall_sides.cpp src/semantic_localization/sensors/pillars.cpp )


target_link_libraries(semantic_localization_map semantic_localization_pf semantic_localization_sensors)

add_library(semantic_localization src/semantic_localization/semantic_localization.cpp)
add_dependencies(semantic_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(semantic_localization
    semantic_localization_sensors semantic_localization_map semantic_localization_pf
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(semantic_localization_node src/semantic_localization/semantic_localization_node.cpp)
add_dependencies(semantic_localization_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(semantic_localization_node
    semantic_localization_sensors semantic_localization_map semantic_localization_pf semantic_localization
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)


install( TARGETS
    semantic_localization_map semantic_localization_pf semantic_localization_sensors
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/semantic_localization/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)