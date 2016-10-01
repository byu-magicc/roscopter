cmake_minimum_required(VERSION 2.8.3)
project(ros_copter)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  cmake_modules
  sensor_msgs
  roscpp
  rospy
  eigen_conversions
  std_msgs
  fcu_common
  dynamic_reconfigure
)
find_package(Eigen REQUIRED)
# find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)


set(CMAKE_CXX_FLAGS "-std=c++0x")

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
  INCLUDE_DIRS include
  LIBRARIES ros_copter
  CATKIN_DEPENDS roscpp rospy nav_msgs std_msgs
)

###########
## Build ##
###########
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

## Declare cpp libraries
add_library(ros_copter
  include/lib/eigen.h
  src/lib/eigen.cpp
)
add_dependencies(ros_copter fcu_common_generate_messages_cpp)
target_link_libraries(ros_copter ${catkin_LIBRARIES})


## Declare cpp executables
add_executable(ekf
  src/ekf/ekf_node.cpp
  src/ekf/ekf.cpp
  include/ekf/ekf.h
)
add_dependencies(ekf fcu_common_generate_messages_cpp)
target_link_libraries(ekf ${catkin_LIBRARIES})

add_executable(controller
  src/controller/controller_node.cpp
  src/controller/controller.cpp
  include/controller/controller.h)
add_dependencies(controller fcu_common_generate_messages_cpp ros_copter_gencfg)
target_link_libraries(controller ${catkin_LIBRARIES})
