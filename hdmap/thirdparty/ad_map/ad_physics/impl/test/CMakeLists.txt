# ----------------- BEGIN LICENSE BLOCK ---------------------------------
#
# Copyright (C) 2019-2020 Intel Corporation
#
# SPDX-License-Identifier: MIT
#
# ----------------- END LICENSE BLOCK -----------------------------------

#####################################################################
# ad-physics-tests - executable setup
#####################################################################

set(EXEC_NAME ad-physics-tests)

include(generated/testfiles.cmake)

set(TEST_SOURCES
  main.cpp
  AngleTests.cpp
  ${GENERATED_TEST_SOURCES}
)

if(BINDING_TYPE STREQUAL ROS)
  catkin_add_gtest(${EXEC_NAME} ${TEST_SOURCES} WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

else()
  add_executable(${EXEC_NAME} ${TEST_SOURCES})

  target_link_libraries(${EXEC_NAME}
    GTest::gtest
  )

  add_test(NAME ${EXEC_NAME} COMMAND ${EXEC_NAME} WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
endif()

target_link_libraries(${EXEC_NAME}
  ${PROJECT_NAME}
)

target_include_directories(${EXEC_NAME}
  PRIVATE
  ../src
)

target_compile_options(${EXEC_NAME} PRIVATE ${TARGET_COMPILE_OPTIONS})
set_property(TARGET ${EXEC_NAME} APPEND_STRING PROPERTY LINK_FLAGS ${TARGET_LINK_FLAGS})