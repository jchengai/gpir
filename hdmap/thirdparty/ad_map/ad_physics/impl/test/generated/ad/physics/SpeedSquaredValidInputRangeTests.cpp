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

#include "ad/physics/SpeedSquaredValidInputRange.hpp"

TEST(SpeedSquaredValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::SpeedSquared value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedSquaredValidInputRangeTests, testValidInputRangeMinOk)
{
  ::ad::physics::SpeedSquared value(-1e6);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedSquaredValidInputRangeTests, testValidInputRangeMaxOk)
{
  ::ad::physics::SpeedSquared value(1e6);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedSquaredValidInputRangeTests, testValidInputRangeBelowMin)
{
  ::ad::physics::SpeedSquared value(::ad::physics::SpeedSquared::cMinValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedSquaredValidInputRangeTests, testValidInputRangeExceedsMax)
{
  ::ad::physics::SpeedSquared value(::ad::physics::SpeedSquared::cMaxValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedSquaredValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::physics::SpeedSquared value(-1e4);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedSquaredValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::physics::SpeedSquared value(1e4);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedSquaredValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::physics::SpeedSquared value(-1e4 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedSquaredValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::physics::SpeedSquared value(1e4 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
