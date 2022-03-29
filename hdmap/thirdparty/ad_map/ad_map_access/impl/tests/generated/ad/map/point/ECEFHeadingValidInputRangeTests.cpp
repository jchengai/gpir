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

#include "ad/map/point/ECEFHeadingValidInputRange.hpp"

TEST(ECEFHeadingValidInputRangeTests, testValidInputRange)
{
  ::ad::map::point::ECEFHeading value;
  ::ad::map::point::ECEFCoordinate valueX(-6400000);
  value.x = valueX;
  ::ad::map::point::ECEFCoordinate valueY(-6400000);
  value.y = valueY;
  ::ad::map::point::ECEFCoordinate valueZ(-6400000);
  value.z = valueZ;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ECEFHeadingValidInputRangeTests, testValidInputRangeXTooSmall)
{
  ::ad::map::point::ECEFHeading value;
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

TEST(ECEFHeadingValidInputRangeTests, testValidInputRangeXTooBig)
{
  ::ad::map::point::ECEFHeading value;
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

TEST(ECEFHeadingValidInputRangeTests, testValidInputRangexDefault)
{
  ::ad::map::point::ECEFHeading value;
  ::ad::map::point::ECEFCoordinate valueDefault;
  value.x = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFHeadingValidInputRangeTests, testValidInputRangeYTooSmall)
{
  ::ad::map::point::ECEFHeading value;
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

TEST(ECEFHeadingValidInputRangeTests, testValidInputRangeYTooBig)
{
  ::ad::map::point::ECEFHeading value;
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

TEST(ECEFHeadingValidInputRangeTests, testValidInputRangeyDefault)
{
  ::ad::map::point::ECEFHeading value;
  ::ad::map::point::ECEFCoordinate valueDefault;
  value.y = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFHeadingValidInputRangeTests, testValidInputRangeZTooSmall)
{
  ::ad::map::point::ECEFHeading value;
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

TEST(ECEFHeadingValidInputRangeTests, testValidInputRangeZTooBig)
{
  ::ad::map::point::ECEFHeading value;
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

TEST(ECEFHeadingValidInputRangeTests, testValidInputRangezDefault)
{
  ::ad::map::point::ECEFHeading value;
  ::ad::map::point::ECEFCoordinate valueDefault;
  value.z = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
