cmake_minimum_required(VERSION 2.8.3)
project(aerial_motion_planning_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  hydrus
  dragon
  message_generation
)

add_service_files(FILES Keyposes.srv)
generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multilink_state
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

Add_library (multilink_state src/multilink_state.cpp )
target_link_libraries (multilink_state ${catkin_LIBRARIES})
