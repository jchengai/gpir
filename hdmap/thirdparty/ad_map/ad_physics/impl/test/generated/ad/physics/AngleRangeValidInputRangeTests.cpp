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

#include "ad/physics/AngleRangeValidInputRange.hpp"

TEST(AngleRangeValidInputRangeTests, testValidInputRange)
{
  ::ad::physics::AngleRange value;
  ::ad::physics::Angle valueMinimum(-6.283185308);
  value.minimum = valueMinimum;
  ::ad::physics::Angle valueMaximum(-6.283185308);
  value.maximum = valueMaximum;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngleRangeValidInputRangeTests, testValidInputRangeMinimumTooSmall)
{
  ::ad::physics::AngleRange value;
  ::ad::physics::Angle valueMinimum(-6.283185308);
  value.minimum = valueMinimum;
  ::ad::physics::Angle valueMaximum(-6.283185308);
  value.maximum = valueMaximum;

  // override member with data type value below input range minimum
  ::ad::physics::Angle invalidInitializedMember(-6.283185308 * 1.1);
  value.minimum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngleRangeValidInputRangeTests, testValidInputRangeMinimumTooBig)
{
  ::ad::physics::AngleRange value;
  ::ad::physics::Angle valueMinimum(-6.283185308);
  value.minimum = valueMinimum;
  ::ad::physics::Angle valueMaximum(-6.283185308);
  value.maximum = valueMaximum;

  // override member with data type value above input range maximum
  ::ad::physics::Angle invalidInitializedMember(6.283185308 * 1.1);
  value.minimum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngleRangeValidInputRangeTests, testValidInputRangeminimumDefault)
{
  ::ad::physics::AngleRange value;
  ::ad::physics::Angle valueDefault;
  value.minimum = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngleRangeValidInputRangeTests, testValidInputRangeMaximumTooSmall)
{
  ::ad::physics::AngleRange value;
  ::ad::physics::Angle valueMinimum(-6.283185308);
  value.minimum = valueMinimum;
  ::ad::physics::Angle valueMaximum(-6.283185308);
  value.maximum = valueMaximum;

  // override member with data type value below input range minimum
  ::ad::physics::Angle invalidInitializedMember(-6.283185308 * 1.1);
  value.maximum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngleRangeValidInputRangeTests, testValidInputRangeMaximumTooBig)
{
  ::ad::physics::AngleRange value;
  ::ad::physics::Angle valueMinimum(-6.283185308);
  value.minimum = valueMinimum;
  ::ad::physics::Angle valueMaximum(-6.283185308);
  value.maximum = valueMaximum;

  // override member with data type value above input range maximum
  ::ad::physics::Angle invalidInitializedMember(6.283185308 * 1.1);
  value.maximum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngleRangeValidInputRangeTests, testValidInputRangemaximumDefault)
{
  ::ad::physics::AngleRange value;
  ::ad::physics::Angle valueDefault;
  value.maximum = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
