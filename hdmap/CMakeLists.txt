cmake_minimum_required(VERSION 3.13.0)
project(hdmap CXX)

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
  roscpp
  common
)

set(ADMAP_INCLUDE_DIRS 
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ad_map/ad_map_access/generated/include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ad_map/ad_map_access/impl/include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ad_map/ad_map_opendrive_reader/include>
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ad_map/ad_physics/generated/include
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ad_map/ad_physics/impl/include
)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/gpir)
include_directories(thirdparty/ad_map/spdlog/install/include)

add_subdirectory(thirdparty/ad_map)
add_subdirectory(pointcloud)
add_subdirectory(utils)
add_subdirectory(road_network)
add_subdirectory(routing)

add_library(hdmap hdmap.cc hdmap_impl.cc)
add_library(hdmap::hdmap ALIAS hdmap)
target_link_libraries(hdmap
  hdmap::pointcloud
  hdmap::road_network
  hdmap::utils
  hdmap::routing
  ad_map_opendrive_reader
  ${catkin_LIBRARIES}
)