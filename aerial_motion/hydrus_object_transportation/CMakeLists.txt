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
#  jsk_mbzirc_board
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hydrus_object_transportation
  CATKIN_DEPENDS aerial_robot_base geometry_msgs hydrus_transform_control roscpp rospy sensor_msgs std_msgs #jsk_mbzirc_board
)

include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
)

Add_library (form_optimization src/form_optimization.cpp )
add_dependencies(form_optimization  ${PROJECT_NAME}_gencpp)
target_link_libraries (form_optimization ${catkin_LIBRARIES}  ${Boost_LIBRARIES}) 

add_executable(form_optimization_node src/form_optimization_node.cpp)
add_dependencies(form_optimization ${PROJECT_NAME}_gencpp)
target_link_libraries (form_optimization_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} form_optimization) 

Add_library (hydrus_object_transportation src/hydrus_object_transportation.cpp )
add_dependencies(hydrus_object_transportation ${PROJECT_NAME}_gencpp)
target_link_libraries (hydrus_object_transportation ${catkin_LIBRARIES}  ${Boost_LIBRARIES}) 

add_executable(hydrus_object_transportation_node src/hydrus_object_transportation_node.cpp)
add_dependencies(hydrus_object_transportation ${PROJECT_NAME}_gencpp)
target_link_libraries (hydrus_object_transportation_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} hydrus_object_transportation) 


