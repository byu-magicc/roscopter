cmake_minimum_required(VERSION 2.8.3)
project(roscopter_utils)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
)
catkin_package(
  INCLUDE_DIRS include lib/geometry/include
)

include_directories(lib/geometry/include include)
