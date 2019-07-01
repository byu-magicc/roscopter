cmake_minimum_required (VERSION 3.7)
project (geometry)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(Eigen3 REQUIRED)

add_library(geometry INTERFACE)
target_include_directories(geometry INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include/>
    ${EIGEN3_INCLUDE_DIRS})

option(BUILD_TESTS "Build Tests" OFF)
if (BUILD_TESTS)
	find_package(GTest REQUIRED)
        find_package(OpenCV REQUIRED)
	add_executable(geometry_test src/test.cpp)
        target_include_directories(geometry_test PUBLIC include ${GTEST_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
        target_link_libraries(geometry_test ${GTEST_LIBRARIES} gtest gtest_main pthread ${OpenCV_LIBRARIES})
endif()


set(LIB_DEST lib/${PROJECT_NAME})
set(INCLUDE_DEST include)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${LIB_DEST}
  LIBRARY DESTINATION ${LIB_DEST}
)
install(DIRECTORY include/${PROJECT_NAME}
  	DESTINATION ${INCLUDE_DEST}
  	FILES_MATCHING PATTERN "*.h"
)

# install CMake package configuration
install(EXPORT ${PROJECT_NAME}-targets DESTINATION ${LIB_DEST})
install(FILES ${PROJECT_NAME}-config.cmake DESTINATION ${LIB_DEST})
