cmake_minimum_required(VERSION 2.8.3)
project(aerial_transportation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aerial_robot_msgs
  aerial_robot_model
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  pluginlib
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS random)

add_subdirectory(qpOASES)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/qpOASES/qpOASES-3.0/bin)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/qpOASES/qpOASES-3.0/include)
add_definitions(-DUSE_QPOASES)

#message(FATAL_ERROR "${CMAKE_CURRENT_BINARY_DIR}")

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

## pluginlibs  library
add_library(aerial_transportation_pluginlib src/grasp_control/base.cpp src/grasp_control/eletromagnet.cpp src/grasp_control/hydrus.cpp)
target_link_libraries(aerial_transportation_pluginlib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(grasp_planning_pluginlib src/grasp_planning/grasp_planner.cpp src/grasp_planning/full_search.cpp src/grasp_planning/bisection_search.cpp)
target_link_libraries(grasp_planning_pluginlib ${catkin_LIBRARIES} ${Boost_LIBRARIES} qpOASES)

add_executable(grasp_planning_node src/grasp_planning_node.cpp)
target_link_libraries (grasp_planning_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

# library of aerial transportation
add_executable(aerial_transportation_node src/aerial_transportation_node.cpp)
target_link_libraries (aerial_transportation_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(object_visualization src/object_visualization.cpp)
target_link_libraries (object_visualization ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

