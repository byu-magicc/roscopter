cmake_minimum_required(VERSION 2.8.3)
project(roscopter)

## Set build type
if (NOT CMAKE_BUILD_TYPE)
  # Options: Debug, Release, MinSizeRel, RelWithDebInfo
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release")
endif()

## Set cmake flags
set(CMAKE_CXX_FLAGS "-std=c++11 -fopenmp")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  eigen_conversions
  rosflight_msgs
  rosflight_utils
  std_msgs
  nav_msgs
  sensor_msgs
  roscopter_common
)
find_package(Eigen3 REQUIRED)

## Dynamically change parameters on the fly
generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS roscpp rospy dynamic_reconfigure eigen_conversions rosflight_msgs rosflight_utils std_msgs nav_msgs sensor_msgs
)

## Specify additional locations of header files
include_directories(
  include 
  ${Eigen3_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)


## Declare cpp executables
add_executable(controller_node src/controller/controller_node.cpp src/controller/controller.cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES})

add_executable(mekf_node src/mekf/mekf_node.cpp src/mekf/mekf.cpp)
target_link_libraries(mekf_node ${catkin_LIBRARIES})