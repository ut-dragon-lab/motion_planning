cmake_minimum_required(VERSION 2.8.3)
project(gap_passing)

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
)

find_package(ompl)

add_message_files(FILES PlanningMode.msg)
add_service_files(FILES Keyposes.srv Endposes.srv)

generate_messages(DEPENDENCIES std_msgs)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES se2_motion_planning
  CATKIN_DEPENDS roscpp  geometry_msgs sensor_msgs tf  moveit_msgs  moveit_core  moveit_ros_planning_interface  pluginlib  hydrus aerial_robot_base message_runtime
)

include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS}
)

add_executable(hydrus_quad_configuration_space_validation_check src/hydrus_quad_configuration_space_validation_check.cpp)
target_link_libraries (hydrus_quad_configuration_space_validation_check ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )


# SE(2)
add_library (se2_motion_control src/se2/motion_control.cpp)
add_dependencies(se2_motion_control  ${PROJECT_NAME}_gencpp)
target_link_libraries (se2_motion_control ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(se2_motion_control_node src/se2/motion_control_node.cpp)
add_dependencies(se2_motion_control  ${PROJECT_NAME}_gencpp)
target_link_libraries (se2_motion_control_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  se2_motion_control ${OMPL_LIBRARIES})

Add_library (se2_motion_planning src/se2/motion_planning.cpp )
add_dependencies(se2_motion_planning  ${PROJECT_NAME}_gencpp)
target_link_libraries (se2_motion_planning ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(se2_motion_planning_node src/se2/motion_planning_node.cpp)
add_dependencies(se2_motion_planning  ${PROJECT_NAME}_gencpp)
target_link_libraries (se2_motion_planning_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} se2_motion_planning se2_motion_control ${OMPL_LIBRARIES})

# SE(3)
add_library (se3_motion_control src/se3/motion_control.cpp)
add_dependencies(se3_motion_control  ${PROJECT_NAME}_gencpp)
target_link_libraries (se3_motion_control ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(se3_motion_control_node src/se3/motion_control_node.cpp)
add_dependencies(se3_motion_control  ${PROJECT_NAME}_gencpp)
target_link_libraries (se3_motion_control_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  se3_motion_control ${OMPL_LIBRARIES})

Add_library (se3_motion_planning src/se3/motion_planning.cpp )
add_dependencies(se3_motion_planning  ${PROJECT_NAME}_gencpp)
target_link_libraries (se3_motion_planning ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(se3_motion_planning_node src/se3/motion_planning_node.cpp)
add_dependencies(se3_motion_planning  ${PROJECT_NAME}_gencpp)
target_link_libraries (se3_motion_planning_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} se3_motion_planning se3_motion_control ${OMPL_LIBRARIES})


install(PROGRAMS
  scripts/state_validation_l4.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
