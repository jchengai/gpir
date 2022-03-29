/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (C) 2018-2020 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

/*
 * Generated file
 */

#include <gtest/gtest.h>

#include <limits>

#include "ad/map/point/ENUEdgeCacheValidInputRange.hpp"

TEST(ENUEdgeCacheValidInputRangeTests, testValidInputRange)
{
  ::ad::map::point::ENUEdgeCache value;
  ::ad::map::point::ENUEdge valueEnuEdge;
  ::ad::map::point::ENUPoint valueEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valueEnuEdgeElementX(-16384);
  valueEnuEdgeElement.x = valueEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valueEnuEdgeElementY(-16384);
  valueEnuEdgeElement.y = valueEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valueEnuEdgeElementZ(-16384);
  valueEnuEdgeElement.z = valueEnuEdgeElementZ;
  valueEnuEdge.resize(1, valueEnuEdgeElement);
  value.enuEdge = valueEnuEdge;
  uint64_t valueEnuVersion{std::numeric_limits<uint64_t>::min()};
  value.enuVersion = valueEnuVersion;
  ASSERT_TRUE(withinValidInputRange(value));
}
