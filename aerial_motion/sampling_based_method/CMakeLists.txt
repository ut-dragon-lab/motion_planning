cmake_minimum_required(VERSION 2.8.3)
project(sampling_based_method)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  tf
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  pluginlib
  hydrus
  dragon
  cmake_modules
  message_generation
  std_msgs
  nav_msgs
  aerial_motion_planning_msgs
)

find_package(ompl)

add_message_files(FILES PlanningMode.msg)

generate_messages(DEPENDENCIES std_msgs)

if("$ENV{ROS_DISTRO}" STREQUAL "debian")
  set(OMPL_INCLUDE_DIRS "/opt/ros/one/include/ompl-1.6")
  set(OMPL_LIBRARIES "/opt/ros/one/lib/x86_64-linux-gnu/libompl.so")
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

catkin_package(
  INCLUDE_DIRS include ${OMPL_INCLUDE_DIRS}
  LIBRARIES motion_planning
  CATKIN_DEPENDS  dragon   moveit_msgs moveit_core moveit_ros_planning_interface
)

include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS}
)


Add_library (motion_planning src/motion_planning.cpp )
add_dependencies(motion_planning  ${PROJECT_NAME}_gencpp)
target_link_libraries (motion_planning ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${OMPL_LIBRARIES})

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
