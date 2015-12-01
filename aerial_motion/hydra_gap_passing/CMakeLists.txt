cmake_minimum_required(VERSION 2.8.3)
project(hydra_gap_passing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  tf
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  pluginlib
  aerial_robot_base
  hydra_transform_control
  cmake_modules
)

find_package(ompl)

add_message_files(FILES PlanningMode.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motion_planning
  CATKIN_DEPENDS roscpp  geometry_msgs sensor_msgs tf  moveit_msgs  moveit_core  moveit_ros_planning_interface  pluginlib  hydra_transform_control   aerial_robot_base message_runtime
)


include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS}
)


add_executable(state_validation_node src/state_validation_node.cpp)
target_link_libraries (state_validation_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ) 




add_library (motion_control src/motion_control.cpp)
add_dependencies(motion_control  ${PROJECT_NAME}_gencpp)
target_link_libraries (motion_control ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${OMPL_LIBRARIES}) 


add_executable(motion_control_node src/motion_control_node.cpp)
add_dependencies(motion_control  ${PROJECT_NAME}_gencpp)
target_link_libraries (motion_control_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES}  motion_control ${OMPL_LIBRARIES}) 


Add_library (motion_planning src/motion_planning.cpp )
add_dependencies(motion_planning  ${PROJECT_NAME}_gencpp)
target_link_libraries (motion_planning ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${OMPL_LIBRARIES}) 

add_executable(motion_planning_node src/motion_planning_node.cpp)
add_dependencies(motion_planning  ${PROJECT_NAME}_gencpp)
target_link_libraries (motion_planning_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} motion_planning motion_control ${OMPL_LIBRARIES}) 


install(PROGRAMS
  scripts/state_validation_l4.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS motion_planning_node motion_planning
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

