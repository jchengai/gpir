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

#include "ad/physics/AngularVelocityValidInputRange.hpp"

TEST(AngularVelocityValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::AngularVelocity value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocityValidInputRangeTests, testValidInputRangeMinOk)
{
  ::ad::physics::AngularVelocity value(-1e3);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocityValidInputRangeTests, testValidInputRangeMaxOk)
{
  ::ad::physics::AngularVelocity value(1e3);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocityValidInputRangeTests, testValidInputRangeBelowMin)
{
  ::ad::physics::AngularVelocity value(::ad::physics::AngularVelocity::cMinValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocityValidInputRangeTests, testValidInputRangeExceedsMax)
{
  ::ad::physics::AngularVelocity value(::ad::physics::AngularVelocity::cMaxValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocityValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::physics::AngularVelocity value(-100.);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngularVelocityValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::physics::AngularVelocity value(100.);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngularVelocityValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::physics::AngularVelocity value(-100. * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngularVelocityValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::physics::AngularVelocity value(100. * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
