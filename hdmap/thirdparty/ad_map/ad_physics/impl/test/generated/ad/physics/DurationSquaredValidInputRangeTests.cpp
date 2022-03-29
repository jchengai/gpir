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

#include "ad/physics/DurationSquaredValidInputRange.hpp"

TEST(DurationSquaredValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::physics::DurationSquared value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DurationSquaredValidInputRangeTests, testValidInputRangeMinOk)
{
  ::ad::physics::DurationSquared value(-1e12);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DurationSquaredValidInputRangeTests, testValidInputRangeMaxOk)
{
  ::ad::physics::DurationSquared value(1e12);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DurationSquaredValidInputRangeTests, testValidInputRangeBelowMin)
{
  ::ad::physics::DurationSquared value(::ad::physics::DurationSquared::cMinValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DurationSquaredValidInputRangeTests, testValidInputRangeExceedsMax)
{
  ::ad::physics::DurationSquared value(::ad::physics::DurationSquared::cMaxValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DurationSquaredValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::physics::DurationSquared value(0.);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DurationSquaredValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::physics::DurationSquared value(10000.);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(DurationSquaredValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::physics::DurationSquared value(0. - ::ad::physics::DurationSquared::cPrecisionValue);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(DurationSquaredValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::physics::DurationSquared value(10000. * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
