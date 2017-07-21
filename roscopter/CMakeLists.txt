cmake_minimum_required(VERSION 2.8.3)
project(roscopter)

set(CMAKE_CXX_STANDARD 11)
if (NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "-fopenmp")

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
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

add_service_files(
  FILES
  AddWaypoint.srv
  RemoveWaypoint.srv
  SetWaypointsFromFile.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES roscopter_eigen
  CATKIN_DEPENDS roscpp rospy dynamic_reconfigure eigen_conversions rosflight_msgs rosflight_utils std_msgs nav_msgs sensor_msgs
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

## Declare cpp libraries
add_library(roscopter_eigen include/lib/eigen.h)
set_target_properties(roscopter_eigen PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(roscopter_eigen ${catkin_EXPORTED_TARGETS})
target_link_libraries(roscopter_eigen ${catkin_LIBRARIES})


## Declare cpp executables
add_executable(controller
  src/controller/controller_node.cpp
  src/controller/controller.cpp
  include/controller/controller.h)
add_dependencies(controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(mekf
  src/mekf/mekf_node.cpp
  src/mekf/mekf.cpp
  src/mekf/mekf_math.cpp
)
add_dependencies(mekf ${catkin_EXPORTED_TARGETS})
target_link_libraries(mekf ${catkin_LIBRARIES})