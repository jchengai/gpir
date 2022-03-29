# ----------------- BEGIN LICENSE BLOCK ---------------------------------
#
# Copyright (C) 2018-2019 Intel Corporation
#
# SPDX-License-Identifier: MIT
#
# ----------------- END LICENSE BLOCK -----------------------------------

find_package(PkgConfig)

# specify the sources files of the Generator Managed Library ad_map_access.
set(ad_map_access_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/src/access/AdMapAccess.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/access/Factory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/access/GeometryStore.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/access/Logging.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/access/Operation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/access/Store.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/access/StoreSerialization.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/config/MapConfigFileHandler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/intersection/Intersection.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/landmark/LandmarkOperation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/lane/BorderOperation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/lane/LaneOperation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/match/AdMapMatching.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/match/MapMatchedOperation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/opendrive/AdMapFactory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/opendrive/DataTypeConversion.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/point/BoundingSphereOperation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/point/CoordinateTransform.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/point/ECEFOperation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/point/ENUOperation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/point/GeometryOperation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/point/GeoOperation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/point/HeadingOperation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/point/Transform.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/restriction/RestrictionOperation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/route/LaneIntervalOperation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/route/Planning.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/route/Route.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/route/RouteAStar.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/route/RouteOperation.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/route/RoutePrediction.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/serialize/ChecksumCRC32.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/serialize/SerializeGeneratedLaneTypes.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/serialize/Serializer.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/serialize/StorageFile.cpp
)

# specify the include directories of the Generator Managed Library ad_map_access.
set(ad_map_access_INCLUDE_DIRS
  ${CMAKE_CURRENT_LIST_DIR}/include/
)

# (optional) add additional entries in target_include_directories()
set(ad_map_access_TARGET_INCLUDE_DIRECTORIES
)

# (optional) add additional entries in target_link_libraries()
set(ad_map_access_TARGET_LINK_LIBRARIES
)

# (optional) specify the unit test directory of the Generator Managed Library ad_map_access.
# The directory needs to contain a CMakeLists.txt
# The tests should not contain any binding-specifics.
set(ad_map_access_UNIT_TEST_DIR
  ${CMAKE_CURRENT_LIST_DIR}/tests
)

# (optional) specify the tools directory of the Generator Managed Library ad_map_access.
# The directory needs to contain a CMakeLists.txt
# The tools might be binding-specific.
#set(ad_map_access_TOOLS_DIR
#  ${CMAKE_CURRENT_LIST_DIR}/tools
#)

