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

#include "ad/physics/Dimension3DValidInputRange.hpp"

TEST(Dimension3DValidInputRangeTests, testValidInputRange)
{
  ::ad::physics::Dimension3D value;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::physics::Distance valueWidth(-1e9);
  value.width = valueWidth;
  ::ad::physics::Distance valueHeight(-1e9);
  value.height = valueHeight;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(Dimension3DValidInputRangeTests, testValidInputRangeLengthTooSmall)
{
  ::ad::physics::Dimension3D value;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::physics::Distance valueWidth(-1e9);
  value.width = valueWidth;
  ::ad::physics::Distance valueHeight(-1e9);
  value.height = valueHeight;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.length = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Dimension3DValidInputRangeTests, testValidInputRangeLengthTooBig)
{
  ::ad::physics::Dimension3D value;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::physics::Distance valueWidth(-1e9);
  value.width = valueWidth;
  ::ad::physics::Distance valueHeight(-1e9);
  value.height = valueHeight;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.length = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Dimension3DValidInputRangeTests, testValidInputRangelengthDefault)
{
  ::ad::physics::Dimension3D value;
  ::ad::physics::Distance valueDefault;
  value.length = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Dimension3DValidInputRangeTests, testValidInputRangeWidthTooSmall)
{
  ::ad::physics::Dimension3D value;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::physics::Distance valueWidth(-1e9);
  value.width = valueWidth;
  ::ad::physics::Distance valueHeight(-1e9);
  value.height = valueHeight;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.width = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Dimension3DValidInputRangeTests, testValidInputRangeWidthTooBig)
{
  ::ad::physics::Dimension3D value;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::physics::Distance valueWidth(-1e9);
  value.width = valueWidth;
  ::ad::physics::Distance valueHeight(-1e9);
  value.height = valueHeight;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.width = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Dimension3DValidInputRangeTests, testValidInputRangewidthDefault)
{
  ::ad::physics::Dimension3D value;
  ::ad::physics::Distance valueDefault;
  value.width = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Dimension3DValidInputRangeTests, testValidInputRangeHeightTooSmall)
{
  ::ad::physics::Dimension3D value;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::physics::Distance valueWidth(-1e9);
  value.width = valueWidth;
  ::ad::physics::Distance valueHeight(-1e9);
  value.height = valueHeight;

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.height = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Dimension3DValidInputRangeTests, testValidInputRangeHeightTooBig)
{
  ::ad::physics::Dimension3D value;
  ::ad::physics::Distance valueLength(-1e9);
  value.length = valueLength;
  ::ad::physics::Distance valueWidth(-1e9);
  value.width = valueWidth;
  ::ad::physics::Distance valueHeight(-1e9);
  value.height = valueHeight;

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.height = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(Dimension3DValidInputRangeTests, testValidInputRangeheightDefault)
{
  ::ad::physics::Dimension3D value;
  ::ad::physics::Distance valueDefault;
  value.height = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
