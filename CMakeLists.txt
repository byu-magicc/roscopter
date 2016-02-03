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
)
find_package(Eigen REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++0x")

add_message_files(
  FILES
  FilterState.msg
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

## Declare cpp libraries
add_library(ros_copter
  include/lib/eigen.h
  src/lib/eigen.cpp
)
add_dependencies(ros_copter ros_copter_generate_messages_cpp fcu_io_generate_messages_cpp)
target_link_libraries(ros_copter ${catkin_LIBRARIES})


## Declare cpp executables
add_executable(ekf
  src/ekf_node.cpp
  src/ekf.cpp
  include/ekf/ekf.h
)
add_dependencies(ekf ros_copter_generate_messages_cpp fcu_io_generate_messages_cpp)
target_link_libraries(ekf ${catkin_LIBRARIES})


add_executable(rc_joy
  src/rc_joy.cpp
  include/rc_joy/rc_joy.h
)
add_dependencies(rc_joy ros_copter_generate_messages_cpp fcu_io_generate_messages_cpp)
target_link_libraries(rc_joy ${catkin_LIBRARIES})
