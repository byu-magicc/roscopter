cmake_minimum_required(VERSION 2.8.3)
project(roscopter_sim)

set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "-fopenmp")

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  rosflight_msgs
  rosflight_utils
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_srvs
  gazebo_ros
  gazebo_plugins
)

find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp tf rosflight_msgs rosflight_utils std_msgs nav_msgs geometry_msgs sensor_msgs std_srvs gazebo_ros gazebo_plugins
  DEPENDS EIGEN3 GAZEBO
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(multirotor_forces_and_moments_plugin src/multirotor_forces_and_moments.cpp)
target_link_libraries(multirotor_forces_and_moments_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(multirotor_forces_and_moments_plugin ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    multirotor_forces_and_moments_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

