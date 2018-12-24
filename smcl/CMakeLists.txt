cmake_minimum_required(VERSION 2.8.3)
project(smcl)

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
    LIBRARIES smcl_map smcl_pf
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(smcl_pf src/smcl/pf/pf.c src/smcl/pf/pf_kdtree.c src/smcl/pf/pf_pdf.c src/smcl/pf/pf_vector.c src/smcl/pf/eig3.c src/smcl/pf/pf_draw.c)

add_library(smcl_map src/smcl/map/semantic_map.c)

add_library(smcl_sensors src/smcl/sensors/sensor.cpp src/smcl/sensors/odom.cpp)

target_link_libraries(smcl_map smcl_pf smcl_sensors)

add_library(smcl src/smcl/smcl.cpp)
add_dependencies(smcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(smcl
    smcl_sensors smcl_map smcl_pf
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(smcl_node src/smcl/smcl_node.cpp)
add_dependencies(smcl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(smcl_node
    smcl_sensors smcl_map smcl_pf smcl
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)


install( TARGETS
    smcl_map smcl_pf smcl_sensors
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/smcl/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)