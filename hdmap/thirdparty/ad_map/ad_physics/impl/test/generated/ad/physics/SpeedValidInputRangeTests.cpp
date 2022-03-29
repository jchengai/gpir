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

#include "ad/physics/SpeedValidInputRange.hpp"

TEST(SpeedValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::Speed value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedValidInputRangeTests, testValidInputRangeMinOk)
{
  ::ad::physics::Speed value(-1e3);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedValidInputRangeTests, testValidInputRangeMaxOk)
{
  ::ad::physics::Speed value(1e3);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedValidInputRangeTests, testValidInputRangeBelowMin)
{
  ::ad::physics::Speed value(::ad::physics::Speed::cMinValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedValidInputRangeTests, testValidInputRangeExceedsMax)
{
  ::ad::physics::Speed value(::ad::physics::Speed::cMaxValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::physics::Speed value(-100.);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::physics::Speed value(100.);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(SpeedValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::physics::Speed value(-100. * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(SpeedValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::physics::Speed value(100. * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
