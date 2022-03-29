# ----------------- BEGIN LICENSE BLOCK ---------------------------------
#
# Copyright (C) 2018-2019 Intel Corporation
#
# SPDX-License-Identifier: MIT
#
# ----------------- END LICENSE BLOCK -----------------------------------

# specify the sources files of the Generator Managed Library ad_map_access_test_support.
set(ad_map_access_test_support_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/src/ArtificialIntersectionTestBase.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/IntersectionTestBase.cpp
)

# specify the include directories of the Generator Managed Library ad_map_access_test_support.
set(ad_map_access_test_support_INCLUDE_DIRS
  ${CMAKE_CURRENT_LIST_DIR}/include/
)
