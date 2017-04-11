cmake_minimum_required(VERSION 2.8.3)
project(victoria_navigation)

#set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )
set (CMAKE_BUILD_TYPE Debug )
SET(FLAGS "-g -ggdb -g3")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED
	geometry_msgs
	gps_common
	roscpp
	roslaunch
	rospy
	sensor_msgs
	std_msgs
	victoria_perception
)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  angles
  geometry_msgs
  gps_common
  roscpp
  rospy
  sensor_msgs
  std_msgs
  DEPENDS yaml-cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(robo_magellan_node
	src/robo_magellan_node.cpp
	src/discover_cone.cpp
	src/move_to_cone.cpp
	src/seek_to_gps.cpp
	src/solve_robomagellan.cpp
	src/strategy_fn.cpp
)

add_dependencies (
	robo_magellan_node
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  robo_magellan_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

roslaunch_add_file_check(launch)

install(
  DIRECTORY config launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

