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

#include "ad/physics/Dimension2DValidInputRange.hpp"

TEST(Dimension2DValidInputRangeTests, testValidInputRange)
{
  ::ad::physics::Dimension2D value;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::physics::Distance valueWidth(-1e9);
  value.width = valueWidth;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Dimension2DValidInputRangeTests, testValidInputRangeLengthTooSmall)
{
  ::ad::physics::Dimension2D value;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::physics::Distance valueWidth(-1e9);
  value.width = valueWidth;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.length = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Dimension2DValidInputRangeTests, testValidInputRangeLengthTooBig)
{
  ::ad::physics::Dimension2D value;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::physics::Distance valueWidth(-1e9);
  value.width = valueWidth;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.length = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Dimension2DValidInputRangeTests, testValidInputRangelengthDefault)
{
  ::ad::physics::Dimension2D value;
  ::ad::physics::Distance valueDefault;
  value.length = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Dimension2DValidInputRangeTests, testValidInputRangeWidthTooSmall)
{
  ::ad::physics::Dimension2D value;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::physics::Distance valueWidth(-1e9);
  value.width = valueWidth;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.width = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Dimension2DValidInputRangeTests, testValidInputRangeWidthTooBig)
{
  ::ad::physics::Dimension2D value;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::physics::Distance valueWidth(-1e9);
  value.width = valueWidth;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.width = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Dimension2DValidInputRangeTests, testValidInputRangewidthDefault)
{
  ::ad::physics::Dimension2D value;
  ::ad::physics::Distance valueDefault;
  value.width = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
