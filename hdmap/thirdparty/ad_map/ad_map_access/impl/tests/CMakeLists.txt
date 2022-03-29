# ----------------- BEGIN LICENSE BLOCK ---------------------------------
#
# Copyright (C) 2018-2019 Intel Corporation
#
# SPDX-License-Identifier: MIT
#
# ----------------- END LICENSE BLOCK -----------------------------------

set(EXEC_NAME ad_map_access_tests)

add_subdirectory(generated)

set(TEST_SOURCES_WITH_PRIVATE_ACCESS
  intersection/SyntheticIntersectionTestBase.cpp
  match/AdMapMatchingTests.cpp
)

set(TEST_SOURCES
  access/AdMapAccessTests.cpp
  access/FactoryTests.cpp
  access/GeometryStoreTests.cpp
  ad_map_access_test_support/src/ArtificialIntersectionTestBase.cpp
  ad_map_access_test_support/src/IntersectionTestBase.cpp
  config/MapConfigFileHandlerTests.cpp
  intersection/MapSetup.cpp
  intersection/MapSetupTests.cpp
  intersection/SyntheticIntersectionTestBase.cpp
  intersection/IntersectionTests.cpp
  intersection/IntersectionTown01PriorityToRightTest.cpp
  intersection/IntersectionPriorityToRightLefthandTests.cpp
  intersection/IntersectionPriorityToRightSingaporeTests.cpp
  intersection/IntersectionPriorityToRightTests.cpp
  intersection/IntersectionYieldLefthandTests.cpp
  intersection/IntersectionYieldTests.cpp
  intersection/IntersectionBasicTrafficLightTest.cpp
  intersection/IntersectionTrafficLightTests.cpp
  intersection/IntersectionTpkPFZYieldTest.cpp
  intersection/IntersectionTpkPFZPriorityToRightTest.cpp
  intersection/IntersectionTpkPFZTrafficLightTest.cpp
  landmark/KnownTrafficSigns.cpp
  landmark/LandmarkOperationTests.cpp
  landmark/TrafficLightTests.cpp
  landmark/TrafficSignTests.cpp
  lane/LaneOperationTests.cpp
  match/AdMapBoundingBoxMapMatchingTest.cpp
  opendrive/OpenDriveAccessTests.cpp
  point/CoordinateTransformTests.cpp
  point/GeometryOperationTests.cpp
  point/GeoOperationTests.cpp
  point/PointOperationTests.cpp
  route/RoutePlanningTests.cpp
  route/RoutePredictionTest.cpp
  route/LaneChangeTests.cpp
  route/LaneIntervalOperationTest.cpp
  serialize/SerializationTests.cpp
  ${TEST_SOURCES_WITH_PRIVATE_ACCESS}
  main.cpp
)
set_source_files_properties(${TEST_SOURCES_WITH_PRIVATE_ACCESS} PROPERITES COMPILE_FLAGS -fno-access-control)

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
