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

#include "ad/physics/DistanceSquaredValidInputRange.hpp"

TEST(DistanceSquaredValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::DistanceSquared value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DistanceSquaredValidInputRangeTests, testValidInputRangeMinOk)
{
  ::ad::physics::DistanceSquared value(-1e18);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DistanceSquaredValidInputRangeTests, testValidInputRangeMaxOk)
{
  ::ad::physics::DistanceSquared value(1e18);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DistanceSquaredValidInputRangeTests, testValidInputRangeBelowMin)
{
  ::ad::physics::DistanceSquared value(::ad::physics::DistanceSquared::cMinValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DistanceSquaredValidInputRangeTests, testValidInputRangeExceedsMax)
{
  ::ad::physics::DistanceSquared value(::ad::physics::DistanceSquared::cMaxValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DistanceSquaredValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::physics::DistanceSquared value(0.);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DistanceSquaredValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::physics::DistanceSquared value(1e12);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DistanceSquaredValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::physics::DistanceSquared value(0. - ::ad::physics::DistanceSquared::cPrecisionValue);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DistanceSquaredValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::physics::DistanceSquared value(1e12 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
