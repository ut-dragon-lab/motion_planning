cmake_minimum_required(VERSION 2.8.3)
project(vectoring_thrust_grasp)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hydrus
  dragon
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include(ExternalProject)

# OSQP
set(OSQP_INSTALL_DIR ${PROJECT_SOURCE_DIR}/osqp)
ExternalProject_Add(OSQP
  DOWNLOAD_COMMAND git clone --recursive https://github.com/oxfordcontrol/osqp OSQP
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${OSQP_INSTALL_DIR}
  TIMEOUT 30
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND ""
  INSTALL_COMMAND make install
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
include_directories(${OSQP_INSTALL_DIR}/include/osqp)
link_directories(${OSQP_INSTALL_DIR}/lib)

# OSQP Eigen Wrapper
set(OSQP_EIGEN_INSTALL_DIR ${PROJECT_SOURCE_DIR}/osqp_eigen)
ExternalProject_Add(OSQP_EIGEN
  DEPENDS OSQP
  GIT_REPOSITORY https://github.com/robotology/osqp-eigen.git
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${OSQP_EIGEN_INSTALL_DIR} -DCMAKE_PROJECT_OsqpEigen_INCLUDE=${OSQP_INSTALL_DIR}/osqp_path.cmake
  TIMEOUT 30
  PATCH_COMMAND patch -s -p0 < ${PROJECT_SOURCE_DIR}/patch/osqp_eigen.patch
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND make install
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
include_directories(${OSQP_EIGEN_INSTALL_DIR}/include)
link_directories(${OSQP_EIGEN_INSTALL_DIR}/lib)

# NLOPT
set(NLOPT_INSTALL_DIR ${PROJECT_SOURCE_DIR}/nlopt)
ExternalProject_Add(NLOPT
  GIT_REPOSITORY https://github.com/stevengj/nlopt.git
  GIT_TAG v2.5.0
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${NLOPT_INSTALL_DIR}
  TIMEOUT 30
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND make install
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
include_directories(${NLOPT_INSTALL_DIR}/include)
link_directories(${NLOPT_INSTALL_DIR}/lib)

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
  ${catkin_INCLUDE_DIRS}
)

add_library(vectoring_thrust_planner
  src/vectoring_thrust_planner.cpp)
target_link_libraries(vectoring_thrust_planner ${catkin_LIBRARIES} osqp OsqpEigen nlopt)
add_dependencies(vectoring_thrust_planner OSQP OSQP_EIGEN NLOPT)

add_executable(vectoring_thrust_test_node src/vectoring_thrust_test_node.cpp)
target_link_libraries(vectoring_thrust_test_node vectoring_thrust_planner ${catkin_LIBRARIES})


add_library(grasping_motion
  src/grasping_motion.cpp)
target_link_libraries(grasping_motion ${catkin_LIBRARIES} vectoring_thrust_planner)

add_executable(grasping_motion_node src/grasping_motion_node.cpp)
target_link_libraries(grasping_motion_node ${catkin_LIBRARIES} grasping_motion)
