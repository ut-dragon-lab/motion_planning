cmake_minimum_required(VERSION 2.8.3)
project(hydrus_object_transportation)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_base
  geometry_msgs
  hydrus_transform_control
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hydrus_object_transportation
  CATKIN_DEPENDS aerial_robot_base geometry_msgs hydrus_transform_control roscpp rospy sensor_msgs std_msgs
)

include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
)

Add_library (posture_optimization src/posture_optimization.cpp )
add_dependencies(posture_optimization  ${PROJECT_NAME}_gencpp)
target_link_libraries (posture_optimization ${catkin_LIBRARIES}  ${Boost_LIBRARIES}) 

add_executable(posture_optimization_node src/posture_optimization_node.cpp)
add_dependencies(posture_optimization ${PROJECT_NAME}_gencpp)
target_link_libraries (posture_optimization_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} posture_optimization) 


