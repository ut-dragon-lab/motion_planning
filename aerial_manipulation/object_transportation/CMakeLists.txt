cmake_minimum_required(VERSION 2.8.3)
project(object_transportation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aerial_robot_msgs
  aerial_robot_base
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  rospy
  std_msgs
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
#  LIBRARIES object_transportation
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
add_library(grasp_pluginlib src/grasp_plugin.cpp)
target_link_libraries(grasp_pluginlib ${catkin_LIBRARIES})

# library of object transportation
add_library(object_transportation src/object_transportation.cpp)
target_link_libraries(object_transportation ${catkin_LIBRARIES})
add_executable(object_transportation_node src/object_transportation_node.cpp)
target_link_libraries (object_transportation_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} object_transportation)
