cmake_minimum_required(VERSION 2.8.3)
project(mocap_filter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  cmake_modules
  sensor_msgs
  roscpp
  rospy
  relative_nav
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
# add_library(mocap_filter
#   src/${PROJECT_NAME}/mocap_filter.cpp
# )

## Declare a cpp executable
add_executable(mocap_filter_node
  src/mocap_filter_node.cpp
  src/mocap_filter.cpp
  include/mocap_filter/mocap_filter.h
  include/mocap_filter/message.h
)

## Add cmake target dependencies of the executable/library
add_dependencies(mocap_filter_node relative_nav_msgs_generate_messages_cpp)

target_link_libraries(mocap_filter_node
  ${catkin_LIBRARIES}
)
