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

#include "ad/physics/SpeedRangeValidInputRange.hpp"

TEST(SpeedRangeValidInputRangeTests, testValidInputRange)
{
  ::ad::physics::SpeedRange value;
  ::ad::physics::Speed valueMinimum(-100.);
  value.minimum = valueMinimum;
  ::ad::physics::Speed valueMaximum(-100.);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedRangeValidInputRangeTests, testValidInputRangeMinimumTooSmall)
{
  ::ad::physics::SpeedRange value;
  ::ad::physics::Speed valueMinimum(-100.);
  value.minimum = valueMinimum;
  ::ad::physics::Speed valueMaximum(-100.);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value below input range minimum
  ::ad::physics::Speed invalidInitializedMember(-100. * 1.1);
  value.minimum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedRangeValidInputRangeTests, testValidInputRangeMinimumTooBig)
{
  ::ad::physics::SpeedRange value;
  ::ad::physics::Speed valueMinimum(-100.);
  value.minimum = valueMinimum;
  ::ad::physics::Speed valueMaximum(-100.);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value above input range maximum
  ::ad::physics::Speed invalidInitializedMember(100. * 1.1);
  value.minimum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedRangeValidInputRangeTests, testValidInputRangeminimumDefault)
{
  ::ad::physics::SpeedRange value;
  ::ad::physics::Speed valueDefault;
  value.minimum = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedRangeValidInputRangeTests, testValidInputRangeMaximumTooSmall)
{
  ::ad::physics::SpeedRange value;
  ::ad::physics::Speed valueMinimum(-100.);
  value.minimum = valueMinimum;
  ::ad::physics::Speed valueMaximum(-100.);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value below input range minimum
  ::ad::physics::Speed invalidInitializedMember(-100. * 1.1);
  value.maximum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedRangeValidInputRangeTests, testValidInputRangeMaximumTooBig)
{
  ::ad::physics::SpeedRange value;
  ::ad::physics::Speed valueMinimum(-100.);
  value.minimum = valueMinimum;
  ::ad::physics::Speed valueMaximum(-100.);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value above input range maximum
  ::ad::physics::Speed invalidInitializedMember(100. * 1.1);
  value.maximum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedRangeValidInputRangeTests, testValidInputRangemaximumDefault)
{
  ::ad::physics::SpeedRange value;
  ::ad::physics::Speed valueDefault;
  value.maximum = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
