cmake_minimum_required(VERSION 3.13.0)
project(planning_core CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED true)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native")

catkin_package()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")
find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy tf tf2 visualization_msgs geometry_msgs
  common
  hdmap
  gp_planner
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/gpir)

set(msgs_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)

add_subdirectory(navigation)
add_subdirectory(planning_common)
add_subdirectory(mock_predictor)
add_subdirectory(simulation)

add_library(planning_core INTERFACE)
target_link_libraries(planning_core INTERFACE
  planning_core::navigation
  planning_core::common
  planning_core::mock_predictor
  planning_core::simulation
)

add_executable(planning
  planning_core.cc
  planning.cc
)
target_link_libraries(planning
  hdmap
  planning_core
  gp_planner
)