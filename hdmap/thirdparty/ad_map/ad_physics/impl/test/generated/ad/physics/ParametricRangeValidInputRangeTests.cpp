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

#include "ad/physics/ParametricRangeValidInputRange.hpp"

TEST(ParametricRangeValidInputRangeTests, testValidInputRange)
{
  ::ad::physics::ParametricRange value;
  ::ad::physics::ParametricValue valueMinimum(0.);
  value.minimum = valueMinimum;
  ::ad::physics::ParametricValue valueMaximum(0.);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ParametricRangeValidInputRangeTests, testValidInputRangeMinimumTooSmall)
{
  ::ad::physics::ParametricRange value;
  ::ad::physics::ParametricValue valueMinimum(0.);
  value.minimum = valueMinimum;
  ::ad::physics::ParametricValue valueMaximum(0.);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value below input range minimum
  ::ad::physics::ParametricValue invalidInitializedMember(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  value.minimum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ParametricRangeValidInputRangeTests, testValidInputRangeMinimumTooBig)
{
  ::ad::physics::ParametricRange value;
  ::ad::physics::ParametricValue valueMinimum(0.);
  value.minimum = valueMinimum;
  ::ad::physics::ParametricValue valueMaximum(0.);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value above input range maximum
  ::ad::physics::ParametricValue invalidInitializedMember(1. * 1.1);
  value.minimum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ParametricRangeValidInputRangeTests, testValidInputRangeminimumDefault)
{
  ::ad::physics::ParametricRange value;
  ::ad::physics::ParametricValue valueDefault;
  value.minimum = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ParametricRangeValidInputRangeTests, testValidInputRangeMaximumTooSmall)
{
  ::ad::physics::ParametricRange value;
  ::ad::physics::ParametricValue valueMinimum(0.);
  value.minimum = valueMinimum;
  ::ad::physics::ParametricValue valueMaximum(0.);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value below input range minimum
  ::ad::physics::ParametricValue invalidInitializedMember(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  value.maximum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ParametricRangeValidInputRangeTests, testValidInputRangeMaximumTooBig)
{
  ::ad::physics::ParametricRange value;
  ::ad::physics::ParametricValue valueMinimum(0.);
  value.minimum = valueMinimum;
  ::ad::physics::ParametricValue valueMaximum(0.);
  value.maximum = valueMaximum;
  value.maximum = value.minimum;
  value.minimum = value.maximum;

  // override member with data type value above input range maximum
  ::ad::physics::ParametricValue invalidInitializedMember(1. * 1.1);
  value.maximum = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ParametricRangeValidInputRangeTests, testValidInputRangemaximumDefault)
{
  ::ad::physics::ParametricRange value;
  ::ad::physics::ParametricValue valueDefault;
  value.maximum = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
