cmake_minimum_required(VERSION 2.8.3)
project(differential_kinematics)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_model
  hydrus
  dragon
  aerial_motion_planning_msgs
  pluginlib)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# QP OASES
add_subdirectory(qpOASES)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/qpOASES/qpOASES-3.0/bin)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/qpOASES/qpOASES-3.0/include)
add_definitions(-DUSE_QPOASES)

# FCL
SET(FCL_BUILD_TESTS OFF CACHE BOOL "Build FCL tests")
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/fcl/include/fcl/fcl.h.in)
  message(WARNING "fcl/include/fcl/fcl.h.in is not exists, download this")
  execute_process(COMMAND git submodule init fcl   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  execute_process(COMMAND git submodule update fcl WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()
if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/fcl)
  add_subdirectory(fcl)
endif()
include_directories(fcl/include
  ${CMAKE_CURRENT_BINARY_DIR}/fcl/include)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/fcl/lib)

add_service_files(
  FILES
  TargetPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include fcl/include ${CMAKE_CURRENT_BINARY_DIR}/fcl/include ${CMAKE_CURRENT_BINARY_DIR}/qpOASES/qpOASES-3.0/include
  LIBRARIES  differential_kinematics_planner_core end_effector_ik_solver_core fcl #differential_kinematics_cost_pluginlib differential_kinematics_constraint_pluginlib
  CATKIN_DEPENDS hydrus dragon
)

# Eigen requires optimization to get good performance
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")


###########
## Build ##
###########

include_directories(
  include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
)

add_library (differential_kinematics_planner_core src/core/planner_core.cpp)
target_link_libraries (differential_kinematics_planner_core ${catkin_LIBRARIES} qpOASES)

# cost plugin
add_library (differential_kinematics_cost_pluginlib
  src/cost/cartesian_constraint.cpp
  src/cost/state_vel.cpp)
target_link_libraries (differential_kinematics_cost_pluginlib ${catkin_LIBRARIES})

# constraint plugin
add_library (differential_kinematics_constraint_pluginlib
  src/constraint/stability.cpp
  src/constraint/collision_avoidance.cpp
  src/constraint/state_limit.cpp
  src/constraint/cog_motion.cpp
  )
target_link_libraries (differential_kinematics_constraint_pluginlib ${catkin_LIBRARIES} fcl)

# se3 specified differential kinematics constraints
add_library (dragon_differential_kinematics_constraint_pluginlib
  src/constraint/se3/link_attitude.cpp
  src/constraint/se3/overlap.cpp)
target_link_libraries (dragon_differential_kinematics_constraint_pluginlib ${catkin_LIBRARIES})

# end-effector IK solver core
add_library (end_effector_ik_solver_core src/motion/end_effector_ik_solver_core.cpp)
target_link_libraries (end_effector_ik_solver_core ${catkin_LIBRARIES} differential_kinematics_planner_core fcl)
add_dependencies(end_effector_ik_solver_core ${PROJECT_NAME}_generate_messages_cpp)

## specific robots:
add_executable (se2_end_effector_ik_node src/motion/sample/se2_end_effector_ik_node.cpp)
target_link_libraries (se2_end_effector_ik_node ${catkin_LIBRARIES} end_effector_ik_solver_core)

add_executable (se3_end_effector_ik_node src/motion/sample/se3_end_effector_ik_node.cpp)
target_link_libraries (se3_end_effector_ik_node ${catkin_LIBRARIES} end_effector_ik_solver_core)
