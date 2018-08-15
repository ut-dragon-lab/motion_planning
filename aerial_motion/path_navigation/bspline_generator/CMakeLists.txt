cmake_minimum_required(VERSION 2.8.3)
project(bspline_generator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(PACKAGE_DEPENDENCIES
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  tf
  sampling_based_method
  aerial_robot_base
  message_generation
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

add_message_files(FILES ControlPoints.msg)
generate_messages(DEPENDENCIES std_msgs)

include(cmake/DownloadTinyspline.cmake)
tinyspline_build(TINYSPLINE_LIBS TINYSPLINE_INCLUDE_DIR)

add_definitions("${TINYSPLINE_DEFINITIONS}")

catkin_package(
  INCLUDE_DIRS include ${TINYSPLINE_INCLUDE_DIR}
  LIBRARIES bspline_generator ${TINYSPLINE_LIBS}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TINYSPLINE_INCLUDE_DIR}
)

add_library(aerial_plannar src/aerial_plannar.cpp src/tinyspline_interface.cpp)
add_dependencies(aerial_plannar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(aerial_plannar ${catkin_LIBRARIES} ${TINYSPLINE_LIBS})

add_executable(aerial_plannar_node src/aerial_plannar_node.cpp)
target_link_libraries(aerial_plannar_node ${catkin_LIBRARIES} aerial_plannar )
