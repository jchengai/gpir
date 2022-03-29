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

#include "ad/map/point/ECEFPointValidInputRange.hpp"

TEST(ECEFPointValidInputRangeTests, testValidInputRange)
{
  ::ad::map::point::ECEFPoint value;
  ::ad::map::point::ECEFCoordinate valueX(-6400000);
  value.x = valueX;
  ::ad::map::point::ECEFCoordinate valueY(-6400000);
  value.y = valueY;
  ::ad::map::point::ECEFCoordinate valueZ(-6400000);
  value.z = valueZ;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ECEFPointValidInputRangeTests, testValidInputRangeXTooSmall)
{
  ::ad::map::point::ECEFPoint value;
  ::ad::map::point::ECEFCoordinate valueX(-6400000);
  value.x = valueX;
  ::ad::map::point::ECEFCoordinate valueY(-6400000);
  value.y = valueY;
  ::ad::map::point::ECEFCoordinate valueZ(-6400000);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::map::point::ECEFCoordinate invalidInitializedMember(-6400000 * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFPointValidInputRangeTests, testValidInputRangeXTooBig)
{
  ::ad::map::point::ECEFPoint value;
  ::ad::map::point::ECEFCoordinate valueX(-6400000);
  value.x = valueX;
  ::ad::map::point::ECEFCoordinate valueY(-6400000);
  value.y = valueY;
  ::ad::map::point::ECEFCoordinate valueZ(-6400000);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::map::point::ECEFCoordinate invalidInitializedMember(6400000 * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFPointValidInputRangeTests, testValidInputRangexDefault)
{
  ::ad::map::point::ECEFPoint value;
  ::ad::map::point::ECEFCoordinate valueDefault;
  value.x = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFPointValidInputRangeTests, testValidInputRangeYTooSmall)
{
  ::ad::map::point::ECEFPoint value;
  ::ad::map::point::ECEFCoordinate valueX(-6400000);
  value.x = valueX;
  ::ad::map::point::ECEFCoordinate valueY(-6400000);
  value.y = valueY;
  ::ad::map::point::ECEFCoordinate valueZ(-6400000);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::map::point::ECEFCoordinate invalidInitializedMember(-6400000 * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFPointValidInputRangeTests, testValidInputRangeYTooBig)
{
  ::ad::map::point::ECEFPoint value;
  ::ad::map::point::ECEFCoordinate valueX(-6400000);
  value.x = valueX;
  ::ad::map::point::ECEFCoordinate valueY(-6400000);
  value.y = valueY;
  ::ad::map::point::ECEFCoordinate valueZ(-6400000);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::map::point::ECEFCoordinate invalidInitializedMember(6400000 * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFPointValidInputRangeTests, testValidInputRangeyDefault)
{
  ::ad::map::point::ECEFPoint value;
  ::ad::map::point::ECEFCoordinate valueDefault;
  value.y = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFPointValidInputRangeTests, testValidInputRangeZTooSmall)
{
  ::ad::map::point::ECEFPoint value;
  ::ad::map::point::ECEFCoordinate valueX(-6400000);
  value.x = valueX;
  ::ad::map::point::ECEFCoordinate valueY(-6400000);
  value.y = valueY;
  ::ad::map::point::ECEFCoordinate valueZ(-6400000);
  value.z = valueZ;

  // override member with data type value below input range minimum
  ::ad::map::point::ECEFCoordinate invalidInitializedMember(-6400000 * 1.1);
  value.z = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFPointValidInputRangeTests, testValidInputRangeZTooBig)
{
  ::ad::map::point::ECEFPoint value;
  ::ad::map::point::ECEFCoordinate valueX(-6400000);
  value.x = valueX;
  ::ad::map::point::ECEFCoordinate valueY(-6400000);
  value.y = valueY;
  ::ad::map::point::ECEFCoordinate valueZ(-6400000);
  value.z = valueZ;

  // override member with data type value above input range maximum
  ::ad::map::point::ECEFCoordinate invalidInitializedMember(6400000 * 1.1);
  value.z = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFPointValidInputRangeTests, testValidInputRangezDefault)
{
  ::ad::map::point::ECEFPoint value;
  ::ad::map::point::ECEFCoordinate valueDefault;
  value.z = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
