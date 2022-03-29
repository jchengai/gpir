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

#include "ad/physics/AccelerationRangeValidInputRange.hpp"

TEST(AccelerationRangeValidInputRangeTests, testValidInputRange)
{
  ::ad::physics::AccelerationRange value;
  ::ad::physics::Acceleration valueMinimum(-1e2);
  value.minimum = valueMinimum;
  ::ad::physics::Acceleration valueMaximum(-1e2);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AccelerationRangeValidInputRangeTests, testValidInputRangeMinimumTooSmall)
{
  ::ad::physics::AccelerationRange value;
  ::ad::physics::Acceleration valueMinimum(-1e2);
  value.minimum = valueMinimum;
  ::ad::physics::Acceleration valueMaximum(-1e2);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value below input range minimum
  ::ad::physics::Acceleration invalidInitializedMember(-1e2 * 1.1);
  value.minimum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AccelerationRangeValidInputRangeTests, testValidInputRangeMinimumTooBig)
{
  ::ad::physics::AccelerationRange value;
  ::ad::physics::Acceleration valueMinimum(-1e2);
  value.minimum = valueMinimum;
  ::ad::physics::Acceleration valueMaximum(-1e2);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value above input range maximum
  ::ad::physics::Acceleration invalidInitializedMember(1e2 * 1.1);
  value.minimum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AccelerationRangeValidInputRangeTests, testValidInputRangeminimumDefault)
{
  ::ad::physics::AccelerationRange value;
  ::ad::physics::Acceleration valueDefault;
  value.minimum = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AccelerationRangeValidInputRangeTests, testValidInputRangeMaximumTooSmall)
{
  ::ad::physics::AccelerationRange value;
  ::ad::physics::Acceleration valueMinimum(-1e2);
  value.minimum = valueMinimum;
  ::ad::physics::Acceleration valueMaximum(-1e2);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value below input range minimum
  ::ad::physics::Acceleration invalidInitializedMember(-1e2 * 1.1);
  value.maximum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AccelerationRangeValidInputRangeTests, testValidInputRangeMaximumTooBig)
{
  ::ad::physics::AccelerationRange value;
  ::ad::physics::Acceleration valueMinimum(-1e2);
  value.minimum = valueMinimum;
  ::ad::physics::Acceleration valueMaximum(-1e2);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value above input range maximum
  ::ad::physics::Acceleration invalidInitializedMember(1e2 * 1.1);
  value.maximum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AccelerationRangeValidInputRangeTests, testValidInputRangemaximumDefault)
{
  ::ad::physics::AccelerationRange value;
  ::ad::physics::Acceleration valueDefault;
  value.maximum = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
