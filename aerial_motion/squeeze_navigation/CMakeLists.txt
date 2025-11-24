cmake_minimum_required(VERSION 2.8.3)
project(squeeze_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_motion_planning_msgs
  bspline_ros
  dragon
  roscpp
  sampling_based_method
  differential_kinematics
  pluginlib
  moveit_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS aerial_motion_planning_msgs bspline_ros dragon roscpp sampling_based_method differential_kinematics
  LIBRARIES planner_pluginlib squeeze_navigation
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

### sequeeze motion planner plugin
add_library(planner_pluginlib
  src/planner/differential_kinematics.cpp
  src/planner/sampling_based_method.cpp)
target_link_libraries(planner_pluginlib ${catkin_LIBRARIES})

add_library(squeeze_navigation src/squeeze_navigation.cpp)
target_link_libraries(squeeze_navigation ${catkin_LIBRARIES} planner_pluginlib)
add_dependencies(squeeze_navigation ${catkin_EXPORTED_TARGETS})

add_executable(squeeze_navigation_node src/squeeze_navigation_node.cpp)
target_link_libraries(squeeze_navigation_node squeeze_navigation ${catkin_LIBRARIES})
