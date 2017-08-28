cmake_minimum_required(VERSION 2.8.3)
project(hydrus_object_transportation)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_base
  geometry_msgs
  hydrus
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  cv_bridge
  )

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11")

generate_dynamic_reconfigure_options(
  config/ObjectDetection.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hydrus_object_transportation
  CATKIN_DEPENDS aerial_robot_base geometry_msgs hydrus roscpp rospy sensor_msgs std_msgs
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

Add_library (object_detection src/object_detection.cpp)
add_dependencies(object_detection ${PROJECT_NAME}_gencfg)
target_link_libraries (object_detection ${catkin_LIBRARIES}) 

add_executable(object_detection_node src/object_detection_node.cpp)
add_dependencies(object_detection_node ${PROJECT_NAME}_gencfg)
target_link_libraries (object_detection_node ${catkin_LIBRARIES} object_detection) 

