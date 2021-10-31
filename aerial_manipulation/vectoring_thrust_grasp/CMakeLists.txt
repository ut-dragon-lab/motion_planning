cmake_minimum_required(VERSION 2.8.3)
project(vectoring_thrust_grasp)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dragon
  roscpp
)

find_package(OsqpEigen REQUIRED)
find_package(NLopt REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES vectoring_thrust_grasp
#  CATKIN_DEPENDS dragon roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OsqpEigen_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(vectoring_thrust_planner  src/vectoring_thrust_planner.cpp)
target_link_libraries(vectoring_thrust_planner ${catkin_LIBRARIES} ${OsqpEigen_LIBRARIES} ${NLOPT_LIBRARIES})

add_executable(vectoring_thrust_test_node src/vectoring_thrust_test_node.cpp)
target_link_libraries(vectoring_thrust_test_node vectoring_thrust_planner ${catkin_LIBRARIES})


add_library(grasping_motion
  src/grasping_motion.cpp)
target_link_libraries(grasping_motion ${catkin_LIBRARIES} vectoring_thrust_planner)

add_executable(grasping_motion_node src/grasping_motion_node.cpp)
target_link_libraries(grasping_motion_node ${catkin_LIBRARIES} grasping_motion)
