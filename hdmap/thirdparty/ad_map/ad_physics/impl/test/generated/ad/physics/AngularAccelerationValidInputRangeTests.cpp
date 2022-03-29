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

#include "ad/physics/AngularAccelerationValidInputRange.hpp"

TEST(AngularAccelerationValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::AngularAcceleration value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularAccelerationValidInputRangeTests, testValidInputRangeMinOk)
{
  ::ad::physics::AngularAcceleration value(-1e3);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularAccelerationValidInputRangeTests, testValidInputRangeMaxOk)
{
  ::ad::physics::AngularAcceleration value(1e3);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularAccelerationValidInputRangeTests, testValidInputRangeBelowMin)
{
  ::ad::physics::AngularAcceleration value(::ad::physics::AngularAcceleration::cMinValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularAccelerationValidInputRangeTests, testValidInputRangeExceedsMax)
{
  ::ad::physics::AngularAcceleration value(::ad::physics::AngularAcceleration::cMaxValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularAccelerationValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::physics::AngularAcceleration value(-1e2);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngularAccelerationValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::physics::AngularAcceleration value(1e2);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngularAccelerationValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::physics::AngularAcceleration value(-1e2 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularAccelerationValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::physics::AngularAcceleration value(1e2 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
