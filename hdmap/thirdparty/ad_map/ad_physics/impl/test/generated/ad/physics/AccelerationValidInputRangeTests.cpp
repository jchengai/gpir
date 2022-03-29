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

#include "ad/physics/AccelerationValidInputRange.hpp"

TEST(AccelerationValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::Acceleration value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AccelerationValidInputRangeTests, testValidInputRangeMinOk)
{
  ::ad::physics::Acceleration value(-1e3);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AccelerationValidInputRangeTests, testValidInputRangeMaxOk)
{
  ::ad::physics::Acceleration value(1e3);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AccelerationValidInputRangeTests, testValidInputRangeBelowMin)
{
  ::ad::physics::Acceleration value(::ad::physics::Acceleration::cMinValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AccelerationValidInputRangeTests, testValidInputRangeExceedsMax)
{
  ::ad::physics::Acceleration value(::ad::physics::Acceleration::cMaxValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AccelerationValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::physics::Acceleration value(-1e2);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AccelerationValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::physics::Acceleration value(1e2);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AccelerationValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::physics::Acceleration value(-1e2 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AccelerationValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::physics::Acceleration value(1e2 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
