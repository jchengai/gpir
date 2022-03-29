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

#include "ad/physics/AngleValidInputRange.hpp"

TEST(AngleValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::Angle value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngleValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::physics::Angle value(-6.283185308);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngleValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::physics::Angle value(6.283185308);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AngleValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::physics::Angle value(-6.283185308 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AngleValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::physics::Angle value(6.283185308 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
