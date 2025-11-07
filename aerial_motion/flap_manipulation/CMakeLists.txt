cmake_minimum_required(VERSION 2.8.3)
project(flap_manipulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  squeeze_navigation
  )

catkin_package(
  INCLUDE_DIRS include
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_library(flap_manipulation src/flap_manipulation.cpp)
target_link_libraries(flap_manipulation ${catkin_LIBRARIES})
add_dependencies(flap_manipulation ${catkin_EXPORTED_TARGETS})

add_executable(flap_manipulation_node src/flap_manipulation_node.cpp)
target_link_libraries(flap_manipulation_node flap_manipulation ${catkin_LIBRARIES})
