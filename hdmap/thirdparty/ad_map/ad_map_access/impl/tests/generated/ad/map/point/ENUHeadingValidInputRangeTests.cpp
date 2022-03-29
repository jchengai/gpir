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

#include "ad/map/point/ENUHeadingValidInputRange.hpp"

TEST(ENUHeadingValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::map::point::ENUHeading value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUHeadingValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::map::point::ENUHeading value(-3.141592655);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ENUHeadingValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::map::point::ENUHeading value(3.141592655);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ENUHeadingValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::map::point::ENUHeading value(-3.141592655 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUHeadingValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::map::point::ENUHeading value(3.141592655 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
