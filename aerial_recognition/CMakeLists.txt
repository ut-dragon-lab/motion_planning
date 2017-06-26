cmake_minimum_required(VERSION 2.8.3)
project(aerial_recognition)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  tf
)
find_package(MRPT REQUIRED slam;vision;gui;topography)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES aerial_recognition
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(laser_detection src/object_detection/laser_detection.cpp)
target_link_libraries(laser_detection  ${MRPT_LIBS} ${catkin_LIBRARIES})

# test for mrpt
add_executable(ransac_sample src/ransac_sample.cpp)
target_link_libraries(ransac_sample ${MRPT_LIBS})

#############
## Install ##
#############

