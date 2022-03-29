# ----------------- BEGIN LICENSE BLOCK ---------------------------------
#
# Copyright (C) 2018-2019 Intel Corporation
#
# SPDX-License-Identifier: MIT
#
# ----------------- END LICENSE BLOCK -----------------------------------

set(EXEC_NAME ad_map_access_gen_tests)

file(GLOB_RECURSE GENERATED_TEST_SOURCES "${CMAKE_CURRENT_LIST_DIR}/*/*.cpp")

set(TEST_SOURCES
  ${GENERATED_TEST_SOURCES}
  main.cpp
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
  ad_map_access_test_support/include
)

target_compile_options(${EXEC_NAME} PRIVATE ${TARGET_COMPILE_OPTIONS})
set_property(TARGET ${EXEC_NAME} APPEND_STRING PROPERTY LINK_FLAGS ${TARGET_LINK_FLAGS})
