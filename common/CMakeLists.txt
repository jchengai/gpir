cmake_minimum_required(VERSION 3.13.0)
project(common CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -march=native")

catkin_package()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")
find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/gpir)

add_subdirectory(utils)
add_subdirectory(solver)
add_subdirectory(frenet)
add_subdirectory(geometry)
add_subdirectory(smoothing)

add_library(common INTERFACE)
target_link_libraries(common INTERFACE
  common::utils
  common::solver
  common::frenet
  common::geometry
  common::smoothing
)