##
# ----------------- BEGIN LICENSE BLOCK ---------------------------------
#
# Copyright (C) 2018-2020 Intel Corporation
#
# SPDX-License-Identifier: MIT
#
# ----------------- END LICENSE BLOCK -----------------------------------
##
##
# Generated file
#
# Generator Version : 11.0.0-1997
##

cmake_minimum_required(VERSION 3.5)
project(ad_physics VERSION 2.4.0)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)

option(BUILD_SHARED_LIBS "Build shared libraries instead of static ones" ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(ad_physics)

if ((NOT ad_physics_SOURCES) OR (NOT ad_physics_INCLUDE_DIRS))
  message(FATAL_ERROR "${PROJECT_NAME}: Variable ad_physics_SOURCES or ad_physics_INCLUDE_DIRS pointing to the generator managed library not set!")
endif()

find_package(spdlog REQUIRED CONFIG)

add_library(${PROJECT_NAME}
  ${ad_physics_SOURCES}
  src/ad/physics/Acceleration.cpp
  src/ad/physics/Angle.cpp
  src/ad/physics/AngularAcceleration.cpp
  src/ad/physics/AngularVelocity.cpp
  src/ad/physics/Distance.cpp
  src/ad/physics/DistanceSquared.cpp
  src/ad/physics/Duration.cpp
  src/ad/physics/DurationSquared.cpp
  src/ad/physics/ParametricValue.cpp
  src/ad/physics/Probability.cpp
  src/ad/physics/RatioValue.cpp
  src/ad/physics/Speed.cpp
  src/ad/physics/SpeedSquared.cpp
  src/ad/physics/Weight.cpp
)

if (DEFINED ad_physics_TARGET_COMPILE_DEFINITIONS)
string(APPEND TARGET_COMPILE_DEFINITIONS ${ad_physics_TARGET_COMPILE_DEFINITIONS})
endif()

target_compile_definitions(${PROJECT_NAME} PUBLIC ${TARGET_COMPILE_DEFINITIONS})
target_compile_options(${PROJECT_NAME} PRIVATE ${TARGET_COMPILE_OPTIONS})
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS ${TARGET_LINK_FLAGS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  ${ad_physics_TARGET_INCLUDE_DIRECTORIES}
  "$<BUILD_INTERFACE:${ad_physics_INCLUDE_DIRS}>"
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  spdlog::spdlog
  ${ad_physics_TARGET_LINK_LIBRARIES}
)

set(BINDING_TYPE MOCKUP)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY ${ad_physics_INCLUDE_DIRS}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}_EXPORT
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set(CMAKECONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

install(EXPORT ${PROJECT_NAME}_EXPORT
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/install/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/install/${PROJECT_NAME}ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/install/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/install/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)

#####################################################################
# Tests
#####################################################################
if (BUILD_TESTING AND ad_physics_UNIT_TEST_DIR)
  add_subdirectory(${ad_physics_UNIT_TEST_DIR} ad_physics_unit_tests)
endif()

#####################################################################
# Tools
#####################################################################
if (ad_physics_TOOLS_DIR)
  add_subdirectory(${ad_physics_TOOLS_DIR} ad_physics_tools)
endif()

