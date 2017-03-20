cmake_minimum_required(VERSION 2.8.3)
project(victoria_navigation)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )
SET(FLAGS "-g -ggdb -g3")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED 
	geometry_msgs
	roscpp
	roslaunch
	rospy
	std_msgs
	victoria_perception
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(cone_finder_strategy_node
    src/cone_finder_strategy_node.cpp
    src/discover_cone.cpp
    src/move_to_cone.cpp
    src/strategy_fn.cpp
)

add_executable(robo_magellan_node
	src/robo_magellan_node.cpp
	src/discover_cone.cpp
	src/move_to_cone.cpp
	src/strategy_fn.cpp
)

add_dependencies (
	robo_magellan_node
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  cone_finder_strategy_node
  ${catkin_LIBRARIES}
)
target_link_libraries(
  robo_magellan_node
  ${catkin_LIBRARIES}
)

roslaunch_add_file_check(launch)

install(
  DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

