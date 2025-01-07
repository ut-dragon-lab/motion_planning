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

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

find_package(Boost REQUIRED COMPONENTS random)

include(ExternalProject)
# QP OASES
set(QPOASES_INSTALL_DIR ${PROJECT_SOURCE_DIR}/qpoases)
ExternalProject_Add(QPOASES
  GIT_REPOSITORY https://github.com/coin-or/qpOASES.git
  GIT_TAG releases/3.2.0
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${QPOASES_INSTALL_DIR}
  TIMEOUT 30
  PATCH_COMMAND
    COMMAND sed -i -e "s/qpOASES\ STATIC/qpOASES\ SHARED/g" CMakeLists.txt
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND make install
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
include_directories(${QPOASES_INSTALL_DIR}/include)
link_directories(${QPOASES_INSTALL_DIR}/lib)

if (NOT EXISTS ${QPOASES_INSTALL_DIR})
  #This has be done in the very first build for
  # catkin_package to be able to find the include folder
  file(MAKE_DIRECTORY ${QPOASES_INSTALL_DIR}/include)
endif()


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
  INCLUDE_DIRS include ${QPOASES_INSTALL_DIR}/include
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
add_dependencies(aerial_transportation_pluginlib QPOASES)

add_library(grasp_planning_pluginlib src/grasp_planning/grasp_planner.cpp src/grasp_planning/full_search.cpp src/grasp_planning/bisection_search.cpp)
target_link_libraries(grasp_planning_pluginlib ${catkin_LIBRARIES} ${Boost_LIBRARIES} qpOASES)
add_dependencies(grasp_planning_pluginlib QPOASES)

add_executable(grasp_planning_node src/grasp_planning_node.cpp)
target_link_libraries (grasp_planning_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
add_dependencies(grasp_planning_node QPOASES)

# library of aerial transportation
add_executable(aerial_transportation_node src/aerial_transportation_node.cpp)
target_link_libraries (aerial_transportation_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(object_visualization src/object_visualization.cpp)
target_link_libraries (object_visualization ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

