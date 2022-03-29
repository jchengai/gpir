# ----------------- BEGIN LICENSE BLOCK ---------------------------------
#
# Copyright (C) 2019-2020 Intel Corporation
#
# SPDX-License-Identifier: MIT
#
# ----------------- END LICENSE BLOCK -----------------------------------

cmake_minimum_required(VERSION 3.5)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

project(ad_map_opendrive_reader VERSION 1.0.0)

option(BUILD_SHARED_LIBS "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)" ON)

set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(PROJECT_SOURCE_LIST
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/GeometryParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/GeoReferenceParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/JunctionParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/LaneParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/OpenDriveParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/ProfilesParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/RoadLinkParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/TrafficGroupParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/TrafficSignalsParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/parser/TrafficSignParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/CenterLine.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/Geometry.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/GeometryGenerator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/geometry/LaneUtils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/OpenDrive.cpp
)

add_library(${PROJECT_NAME} ${PROJECT_SOURCE_LIST})

#####################################################################
# dependencies
#####################################################################

find_package(Boost REQUIRED)

#pugixml
find_library(PUGIXML_LIBRARIES
  NAMES pugixml
)
find_path(PUGIXML_INCLUDE_DIR
  NAMES pugixml.hpp
)

if((NOT PUGIXML_LIBRARIES) OR (NOT PUGIXML_INCLUDE_DIR))
  message(FATAL "Dependency pugixml not found!")
endif()

find_library(PROJ_LIBRARIES
  NAMES proj
)
find_path(PROJ_INCLUDE_DIR
  NAMES proj_api.h
)

if((NOT PROJ_LIBRARIES) OR (NOT PROJ_INCLUDE_DIR))
  message(FATAL "Dependency proj not found!")
endif()

target_include_directories(${PROJECT_NAME}
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PUGIXML_INCLUDE_DIR}
  ${PROJ_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
  ${PUGIXML_LIBRARIES}
  ${PROJ_LIBRARIES}
)


set(PROJECT_PUBLIC_HEADER_LIST
  "${CMAKE_CURRENT_SOURCE_DIR}/include/opendrive/types.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/opendrive/OpenDrive.hpp"
)

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${PROJECT_PUBLIC_HEADER_LIST}")

#####################################################################
# installation
#####################################################################

if(IS_ABSOLUTE "${CMAKE_INSTALL_LIBDIR}")
  file(RELATIVE_PATH CMAKE_INSTALL_LIBDIR_RELATIVE "${CMAKE_INSTALL_PREFIX}" "${CMAKE_INSTALL_LIBDIR}")
else()
  set(CMAKE_INSTALL_LIBDIR_RELATIVE "${CMAKE_INSTALL_LIBDIR}")
endif()

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}_EXPORT
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR_RELATIVE}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR_RELATIVE}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/opendrive
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

set(CMAKECONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR_RELATIVE}/cmake/${PROJECT_NAME}")

install(EXPORT ${PROJECT_NAME}_EXPORT
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/install/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/install/${PROJECT_NAME}Config.cmake"
  DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)
