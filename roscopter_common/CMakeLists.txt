cmake_minimum_required(VERSION 2.8.3)
project(roscopter_common)

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
  std_msgs
  tf
)
find_package(Eigen3 REQUIRED)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roscopter_common
  CATKIN_DEPENDS roscpp std_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
add_library(roscopter_common src/common.cpp)
target_link_libraries(roscopter_common ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

install(TARGETS roscopter_common
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN "*.svn" EXCLUDE
)
