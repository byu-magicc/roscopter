cmake_minimum_required(VERSION 2.8.3)
project(roscopter_utils)

find_package(catkin REQUIRED COMPONENTS
    rospy
)
catkin_package(
  INCLUDE_DIRS include lib
)

include_directories(lib include)
