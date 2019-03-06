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

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES se2_motion_planning se3_motion_planning
)

include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS}
)


# SE(2)
Add_library (se2_motion_planning src/se2/motion_planning.cpp )
add_dependencies(se2_motion_planning  ${PROJECT_NAME}_gencpp)
target_link_libraries (se2_motion_planning ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${OMPL_LIBRARIES})

# SE(3)
Add_library (se3_motion_planning src/se3/motion_planning.cpp )
add_dependencies(se3_motion_planning  ${PROJECT_NAME}_gencpp)
target_link_libraries (se3_motion_planning se2_motion_planning ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${OMPL_LIBRARIES})

install(PROGRAMS
  scripts/state_validation_l4.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
