cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

find_package(Eigen)
#rosbuild_find_ros_package("wave_propagation")
include_directories(${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
#include_directories(${EIGEN_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} "${wave_propagation_PACKAGE_PATH}/include")
#rosbuild_add_library("${wave_propagation_PACKAGE_PATH}/lib" src/2d_path.cpp)
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_boost_directories()

# 2d path
rosbuild_add_library(path2d src/2d_path.cpp)
rosbuild_add_executable(path_2d src/2d_path_node.cpp)
target_link_libraries(path_2d path2d)

rosbuild_add_executable(path_client src/path_client.cpp)


