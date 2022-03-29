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

#include "ad/physics/DurationValidInputRange.hpp"

TEST(DurationValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::Duration value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DurationValidInputRangeTests, testValidInputRangeMinOk)
{
  ::ad::physics::Duration value(-1e6);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DurationValidInputRangeTests, testValidInputRangeMaxOk)
{
  ::ad::physics::Duration value(1e6);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DurationValidInputRangeTests, testValidInputRangeBelowMin)
{
  ::ad::physics::Duration value(::ad::physics::Duration::cMinValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DurationValidInputRangeTests, testValidInputRangeExceedsMax)
{
  ::ad::physics::Duration value(::ad::physics::Duration::cMaxValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DurationValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::physics::Duration value(0.);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DurationValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::physics::Duration value(1e6);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DurationValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::physics::Duration value(0. - ::ad::physics::Duration::cPrecisionValue);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DurationValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::physics::Duration value(1e6 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
