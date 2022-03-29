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

#include "ad/physics/Distance2DValidInputRange.hpp"

TEST(Distance2DValidInputRangeTests, testValidInputRange)
{
  ::ad::physics::Distance2D value;
  ::ad::physics::Distance valueX(-1e9);
  value.x = valueX;
  ::ad::physics::Distance valueY(-1e9);
  value.y = valueY;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Distance2DValidInputRangeTests, testValidInputRangeXTooSmall)
{
  ::ad::physics::Distance2D value;
  ::ad::physics::Distance valueX(-1e9);
  value.x = valueX;
  ::ad::physics::Distance valueY(-1e9);
  value.y = valueY;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Distance2DValidInputRangeTests, testValidInputRangeXTooBig)
{
  ::ad::physics::Distance2D value;
  ::ad::physics::Distance valueX(-1e9);
  value.x = valueX;
  ::ad::physics::Distance valueY(-1e9);
  value.y = valueY;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.x = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Distance2DValidInputRangeTests, testValidInputRangexDefault)
{
  ::ad::physics::Distance2D value;
  ::ad::physics::Distance valueDefault;
  value.x = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Distance2DValidInputRangeTests, testValidInputRangeYTooSmall)
{
  ::ad::physics::Distance2D value;
  ::ad::physics::Distance valueX(-1e9);
  value.x = valueX;
  ::ad::physics::Distance valueY(-1e9);
  value.y = valueY;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Distance2DValidInputRangeTests, testValidInputRangeYTooBig)
{
  ::ad::physics::Distance2D value;
  ::ad::physics::Distance valueX(-1e9);
  value.x = valueX;
  ::ad::physics::Distance valueY(-1e9);
  value.y = valueY;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.y = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Distance2DValidInputRangeTests, testValidInputRangeyDefault)
{
  ::ad::physics::Distance2D value;
  ::ad::physics::Distance valueDefault;
  value.y = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
