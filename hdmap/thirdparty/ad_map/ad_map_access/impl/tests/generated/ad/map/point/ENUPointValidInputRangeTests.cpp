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

#include "ad/map/point/ENUPointValidInputRange.hpp"

TEST(ENUPointValidInputRangeTests, testValidInputRange)
{
  ::ad::map::point::ENUPoint value;
  ::ad::map::point::ENUCoordinate valueX(-16384);
  value.x = valueX;
  ::ad::map::point::ENUCoordinate valueY(-16384);
  value.y = valueY;
  ::ad::map::point::ENUCoordinate valueZ(-16384);
  value.z = valueZ;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ENUPointValidInputRangeTests, testValidInputRangeXTooSmall)
{
  ::ad::map::point::ENUPoint value;
  ::ad::map::point::ENUCoordinate valueX(-16384);
  value.x = valueX;
  ::ad::map::point::ENUCoordinate valueY(-16384);
  value.y = valueY;
  ::ad::map::point::ENUCoordinate valueZ(-16384);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::map::point::ENUCoordinate invalidInitializedMember(-16384 * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUPointValidInputRangeTests, testValidInputRangeXTooBig)
{
  ::ad::map::point::ENUPoint value;
  ::ad::map::point::ENUCoordinate valueX(-16384);
  value.x = valueX;
  ::ad::map::point::ENUCoordinate valueY(-16384);
  value.y = valueY;
  ::ad::map::point::ENUCoordinate valueZ(-16384);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::map::point::ENUCoordinate invalidInitializedMember(16384 * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUPointValidInputRangeTests, testValidInputRangexDefault)
{
  ::ad::map::point::ENUPoint value;
  ::ad::map::point::ENUCoordinate valueDefault;
  value.x = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUPointValidInputRangeTests, testValidInputRangeYTooSmall)
{
  ::ad::map::point::ENUPoint value;
  ::ad::map::point::ENUCoordinate valueX(-16384);
  value.x = valueX;
  ::ad::map::point::ENUCoordinate valueY(-16384);
  value.y = valueY;
  ::ad::map::point::ENUCoordinate valueZ(-16384);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::map::point::ENUCoordinate invalidInitializedMember(-16384 * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUPointValidInputRangeTests, testValidInputRangeYTooBig)
{
  ::ad::map::point::ENUPoint value;
  ::ad::map::point::ENUCoordinate valueX(-16384);
  value.x = valueX;
  ::ad::map::point::ENUCoordinate valueY(-16384);
  value.y = valueY;
  ::ad::map::point::ENUCoordinate valueZ(-16384);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::map::point::ENUCoordinate invalidInitializedMember(16384 * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUPointValidInputRangeTests, testValidInputRangeyDefault)
{
  ::ad::map::point::ENUPoint value;
  ::ad::map::point::ENUCoordinate valueDefault;
  value.y = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUPointValidInputRangeTests, testValidInputRangeZTooSmall)
{
  ::ad::map::point::ENUPoint value;
  ::ad::map::point::ENUCoordinate valueX(-16384);
  value.x = valueX;
  ::ad::map::point::ENUCoordinate valueY(-16384);
  value.y = valueY;
  ::ad::map::point::ENUCoordinate valueZ(-16384);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::map::point::ENUCoordinate invalidInitializedMember(-16384 * 1.1);
  value.z = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUPointValidInputRangeTests, testValidInputRangeZTooBig)
{
  ::ad::map::point::ENUPoint value;
  ::ad::map::point::ENUCoordinate valueX(-16384);
  value.x = valueX;
  ::ad::map::point::ENUCoordinate valueY(-16384);
  value.y = valueY;
  ::ad::map::point::ENUCoordinate valueZ(-16384);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::map::point::ENUCoordinate invalidInitializedMember(16384 * 1.1);
  value.z = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUPointValidInputRangeTests, testValidInputRangezDefault)
{
  ::ad::map::point::ENUPoint value;
  ::ad::map::point::ENUCoordinate valueDefault;
  value.z = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
