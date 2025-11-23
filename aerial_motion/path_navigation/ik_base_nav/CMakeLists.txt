cmake_minimum_required(VERSION 3.0.2)
project(ik_base_nav)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_motion_planning_msgs
  bspline_ros
  differential_kinematics
  dragon
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS aerial_motion_planning_msgs bspline_ros dragon roscpp differential_kinematics
  LIBRARIES dragon_navigation
)


###########
## Build ##
###########

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(dragon_navigation src/dragon_navigation.cpp)
target_link_libraries(dragon_navigation ${catkin_LIBRARIES})
add_dependencies(dragon_navigation ${catkin_EXPORTED_TARGETS})

add_executable(dragon_navigation_node src/dragon_navigation_node.cpp)
target_link_libraries(dragon_navigation_node dragon_navigation ${catkin_LIBRARIES})
