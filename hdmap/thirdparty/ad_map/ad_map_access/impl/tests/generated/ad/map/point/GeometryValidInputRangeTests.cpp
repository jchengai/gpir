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

#include "ad/map/point/GeometryValidInputRange.hpp"

TEST(GeometryValidInputRangeTests, testValidInputRange)
{
  ::ad::map::point::Geometry value;
  bool valueIsValid{true};
  value.isValid = valueIsValid;
  bool valueIsClosed{true};
  value.isClosed = valueIsClosed;
  ::ad::map::point::ECEFEdge valueEcefEdge;
  ::ad::map::point::ECEFPoint valueEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueEcefEdgeElementX(-6400000);
  valueEcefEdgeElement.x = valueEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueEcefEdgeElementY(-6400000);
  valueEcefEdgeElement.y = valueEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueEcefEdgeElementZ(-6400000);
  valueEcefEdgeElement.z = valueEcefEdgeElementZ;
  valueEcefEdge.resize(1, valueEcefEdgeElement);
  value.ecefEdge = valueEcefEdge;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::map::point::ENUEdgeCache valuePrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valuePrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valuePrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valuePrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valuePrivate_enuEdgeCacheEnuEdgeElement.x = valuePrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valuePrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valuePrivate_enuEdgeCacheEnuEdgeElement.y = valuePrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valuePrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valuePrivate_enuEdgeCacheEnuEdgeElement.z = valuePrivate_enuEdgeCacheEnuEdgeElementZ;
  valuePrivate_enuEdgeCacheEnuEdge.resize(1, valuePrivate_enuEdgeCacheEnuEdgeElement);
  valuePrivate_enuEdgeCache.enuEdge = valuePrivate_enuEdgeCacheEnuEdge;
  uint64_t valuePrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valuePrivate_enuEdgeCache.enuVersion = valuePrivate_enuEdgeCacheEnuVersion;
  value.private_enuEdgeCache = valuePrivate_enuEdgeCache;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(GeometryValidInputRangeTests, testValidInputRangeLengthTooSmall)
{
  ::ad::map::point::Geometry value;
  bool valueIsValid{true};
  value.isValid = valueIsValid;
  bool valueIsClosed{true};
  value.isClosed = valueIsClosed;
  ::ad::map::point::ECEFEdge valueEcefEdge;
  ::ad::map::point::ECEFPoint valueEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueEcefEdgeElementX(-6400000);
  valueEcefEdgeElement.x = valueEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueEcefEdgeElementY(-6400000);
  valueEcefEdgeElement.y = valueEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueEcefEdgeElementZ(-6400000);
  valueEcefEdgeElement.z = valueEcefEdgeElementZ;
  valueEcefEdge.resize(1, valueEcefEdgeElement);
  value.ecefEdge = valueEcefEdge;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::map::point::ENUEdgeCache valuePrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valuePrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valuePrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valuePrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valuePrivate_enuEdgeCacheEnuEdgeElement.x = valuePrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valuePrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valuePrivate_enuEdgeCacheEnuEdgeElement.y = valuePrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valuePrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valuePrivate_enuEdgeCacheEnuEdgeElement.z = valuePrivate_enuEdgeCacheEnuEdgeElementZ;
  valuePrivate_enuEdgeCacheEnuEdge.resize(1, valuePrivate_enuEdgeCacheEnuEdgeElement);
  valuePrivate_enuEdgeCache.enuEdge = valuePrivate_enuEdgeCacheEnuEdge;
  uint64_t valuePrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valuePrivate_enuEdgeCache.enuVersion = valuePrivate_enuEdgeCacheEnuVersion;
  value.private_enuEdgeCache = valuePrivate_enuEdgeCache;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.length = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(GeometryValidInputRangeTests, testValidInputRangeLengthTooBig)
{
  ::ad::map::point::Geometry value;
  bool valueIsValid{true};
  value.isValid = valueIsValid;
  bool valueIsClosed{true};
  value.isClosed = valueIsClosed;
  ::ad::map::point::ECEFEdge valueEcefEdge;
  ::ad::map::point::ECEFPoint valueEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate valueEcefEdgeElementX(-6400000);
  valueEcefEdgeElement.x = valueEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate valueEcefEdgeElementY(-6400000);
  valueEcefEdgeElement.y = valueEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate valueEcefEdgeElementZ(-6400000);
  valueEcefEdgeElement.z = valueEcefEdgeElementZ;
  valueEcefEdge.resize(1, valueEcefEdgeElement);
  value.ecefEdge = valueEcefEdge;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::map::point::ENUEdgeCache valuePrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge valuePrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint valuePrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate valuePrivate_enuEdgeCacheEnuEdgeElementX(-16384);
  valuePrivate_enuEdgeCacheEnuEdgeElement.x = valuePrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate valuePrivate_enuEdgeCacheEnuEdgeElementY(-16384);
  valuePrivate_enuEdgeCacheEnuEdgeElement.y = valuePrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate valuePrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
  valuePrivate_enuEdgeCacheEnuEdgeElement.z = valuePrivate_enuEdgeCacheEnuEdgeElementZ;
  valuePrivate_enuEdgeCacheEnuEdge.resize(1, valuePrivate_enuEdgeCacheEnuEdgeElement);
  valuePrivate_enuEdgeCache.enuEdge = valuePrivate_enuEdgeCacheEnuEdge;
  uint64_t valuePrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
  valuePrivate_enuEdgeCache.enuVersion = valuePrivate_enuEdgeCacheEnuVersion;
  value.private_enuEdgeCache = valuePrivate_enuEdgeCache;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.length = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(GeometryValidInputRangeTests, testValidInputRangelengthDefault)
{
  ::ad::map::point::Geometry value;
  ::ad::physics::Distance valueDefault;
  value.length = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
