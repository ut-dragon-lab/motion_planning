cmake_minimum_required(VERSION 2.8.3)
project(bspline_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

set(PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

include(ExternalProject)

# tinyspline for B-Spline
set(TINYSPLINE_INSTALL_DIR ${PROJECT_SOURCE_DIR}/tinyspline)
ExternalProject_Add(tinyspline
  GIT_REPOSITORY https://github.com/msteinbeck/tinyspline.git
  GIT_TAG d536e842f71f997ddb0f11d1d7a1a0d8d3e557d0
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${TINYSPLINE_INSTALL_DIR}
  TIMEOUT 30
  PATCH_COMMAND
    COMMAND sed -i -e "s/VERSION\ 3.4/VERSION\ 2.8/g" CMakeLists.txt
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release
  BUILD_COMMAND make
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND make install
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  )
include_directories(${TINYSPLINE_INSTALL_DIR}/include)
link_directories(${TINYSPLINE_INSTALL_DIR}/lib64)

if (NOT EXISTS ${TINYSPLINE_INSTALL_DIR})
  #This has be done in the very first build for
  # catkin_package to be able to find the include folder
  file(MAKE_DIRECTORY ${TINYSPLINE_INSTALL_DIR}/include)
endif()

catkin_package(
  INCLUDE_DIRS include ${TINYSPLINE_INSTALL_DIR}/include
  LIBRARIES bspline_ros
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
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

add_library(bspline_ros src/bspline_ros.cpp)
add_dependencies(bspline_ros tinyspline)
target_link_libraries(bspline_ros ${catkin_LIBRARIES} tinysplinecxx)
