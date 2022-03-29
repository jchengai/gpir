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

#include "ad/physics/DistanceValidInputRange.hpp"

TEST(DistanceValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::Distance value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DistanceValidInputRangeTests, testValidInputRangeMinOk)
{
  ::ad::physics::Distance value(-1e9);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DistanceValidInputRangeTests, testValidInputRangeMaxOk)
{
  ::ad::physics::Distance value(1e9);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DistanceValidInputRangeTests, testValidInputRangeBelowMin)
{
  ::ad::physics::Distance value(::ad::physics::Distance::cMinValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DistanceValidInputRangeTests, testValidInputRangeExceedsMax)
{
  ::ad::physics::Distance value(::ad::physics::Distance::cMaxValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DistanceValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::physics::Distance value(-1e9);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DistanceValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::physics::Distance value(1e9);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DistanceValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::physics::Distance value(-1e9 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DistanceValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::physics::Distance value(1e9 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
