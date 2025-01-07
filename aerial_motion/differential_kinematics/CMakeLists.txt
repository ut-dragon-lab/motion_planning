cmake_minimum_required(VERSION 2.8.3)
project(differential_kinematics)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  aerial_robot_model
  bspline_ros
  fcl_catkin
  hydrus
  dragon
  aerial_motion_planning_msgs
  pluginlib
  tf_conversions
  kdl_conversions)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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

find_package(fcl_catkin)
string(FIND "${fcl_catkin_INCLUDE_DIRS}" "fcl_catkin" OUTPUT)
if (${OUTPUT} EQUAL -1)
  set(FCL_INCLUDE_DIRS ${fcl_catkin_INCLUDE_DIRS}/fcl_catkin)
else ()
  set(FCL_INCLUDE_DIRS ${fcl_catkin_INCLUDE_DIRS})
endif ()
include_directories(${FCL_INCLUDE_DIRS})

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
  INCLUDE_DIRS include ${QPOASES_INSTALL_DIR}/include ${FCL_INCLUDE_DIRS}
  LIBRARIES  differential_kinematics_planner_core end_effector_ik_solver_core
  CATKIN_DEPENDS hydrus dragon kdl_conversions
  DEPENDS fcl_catkin
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
add_dependencies(differential_kinematics_planner_core QPOASES)

# cost plugin
add_library (differential_kinematics_cost_pluginlib
  src/cost/cartesian_constraint.cpp
  src/cost/state_vel.cpp)
target_link_libraries (differential_kinematics_cost_pluginlib ${catkin_LIBRARIES} differential_kinematics_planner_core)

# constraint plugin
add_library (differential_kinematics_constraint_pluginlib
  src/constraint/static_thrust.cpp
  src/constraint/stability.cpp
  src/constraint/collision_avoidance.cpp
  src/constraint/state_limit.cpp
  src/constraint/cog_motion.cpp
  src/constraint/joint_torque.cpp
  )
target_link_libraries (differential_kinematics_constraint_pluginlib ${catkin_LIBRARIES} differential_kinematics_planner_core ${fcl_catkin_LIBRARIES})

# se3 specified differential kinematics constraints
add_library (dragon_differential_kinematics_constraint_pluginlib
  src/constraint/se3/link_attitude.cpp
  src/constraint/se3/overlap.cpp)
target_link_libraries (dragon_differential_kinematics_constraint_pluginlib ${catkin_LIBRARIES} differential_kinematics_planner_core)

# end-effector IK solver core
add_library (end_effector_ik_solver_core src/motion/end_effector_ik_solver_core.cpp)
target_link_libraries (end_effector_ik_solver_core ${catkin_LIBRARIES} differential_kinematics_planner_core)
add_dependencies(end_effector_ik_solver_core ${PROJECT_NAME}_generate_messages_cpp)

## specific robots:
add_executable (se2_end_effector_ik_node src/motion/sample/se2_end_effector_ik_node.cpp)
target_link_libraries (se2_end_effector_ik_node ${catkin_LIBRARIES} end_effector_ik_solver_core)

add_executable (se3_end_effector_ik_node src/motion/sample/se3_end_effector_ik_node.cpp)
target_link_libraries (se3_end_effector_ik_node ${catkin_LIBRARIES} end_effector_ik_solver_core)
