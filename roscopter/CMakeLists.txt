cmake_minimum_required(VERSION 2.8.3)
project(roscopter)

set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "-fopenmp")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  rosflight_msgs
  rosflight_utils
  std_msgs
  nav_msgs
  sensor_msgs
  rosbag
  roslib
)
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy dynamic_reconfigure eigen_conversions
    rosflight_msgs rosflight_utils std_msgs nav_msgs sensor_msgs roslib
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(include 
  ${EIGEN3_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  )

## Declare cpp executables
add_executable(controller
  src/controller/controller_node.cpp
  src/controller/controller.cpp
  include/controller/controller.h
  src/controller/simple_pid.cpp)
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})

add_library(ekf
  src/ekf/ekf.cpp
  src/ekf/ekf_dyn.cpp
  src/ekf/ekf_meas.cpp
  src/ekf/ekf_helper.cpp
  src/ekf/ekf_log.cpp
)
target_include_directories(ekf PUBLIC lib/geometry/include)
target_link_libraries(ekf)

add_library(ekf_ros src/ekf/ekf_ros.cpp)
target_link_libraries(ekf_ros ekf ${catkin_LIBRARIES})
add_dependencies(ekf_ros ${catkin_EXPORTED_TARGETS})

add_executable(ekf_node src/ekf/ekf_node.cpp)
target_link_libraries(ekf_node ekf_ros ${catkin_LIBRARIES})

add_executable(ekf_rosbag src/ekf/ekf_rosbag.cpp)
target_link_libraries(ekf_rosbag ekf_ros ${catkin_LIBRARIES})
add_dependencies(ekf_rosbag ${catkin_EXPORTED_TARGETS})
