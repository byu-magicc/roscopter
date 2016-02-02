cmake_minimum_required(VERSION 2.8.3)
project(ros_copter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  cmake_modules
  sensor_msgs
  roscpp
  rospy
  relative_nav
  eigen_conversions
)
find_package(Eigen REQUIRED)

catkin_package()

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(ekf
#   src/${PROJECT_NAME}/ekf.cpp
# )

## Declare a cpp executable
add_executable(ekf
  src/ekf_node.cpp
  src/ekf.cpp
  include/ekf/ekf.h
)
add_dependencies(ekf relative_nav_generate_messages_cpp)
target_link_libraries(ekf  ${catkin_LIBRARIES} )
