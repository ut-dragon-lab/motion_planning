cmake_minimum_required(VERSION 2.8.3)
project(aerial_transportation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aerial_robot_base
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  rospy
  std_msgs
  dynamixel_msgs
  sensor_msgs
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES aerial_transportation
#  CATKIN_DEPENDS aeiral_robot_msgs aerial_robot_base dynamic_reconfigure geometry_msgs nav_msgs pluginlib roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
)

## pluginlib_tutorials library
add_library(aerial_transportation_pluginlib src/base.cpp src/method/eletromagnet.cpp src/method/hydrus.cpp)
target_link_libraries(aerial_transportation_pluginlib ${catkin_LIBRARIES})

# library of aerial transportation
add_executable(aerial_transportation_node src/aerial_transportation_node.cpp)
target_link_libraries (aerial_transportation_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )
