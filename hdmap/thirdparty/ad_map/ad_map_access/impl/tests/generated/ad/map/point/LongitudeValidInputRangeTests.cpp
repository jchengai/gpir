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

#include "ad/map/point/LongitudeValidInputRange.hpp"

TEST(LongitudeValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::map::point::Longitude value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LongitudeValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::map::point::Longitude value(-180);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LongitudeValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::map::point::Longitude value(180);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LongitudeValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::map::point::Longitude value(-180 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LongitudeValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::map::point::Longitude value(180 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
