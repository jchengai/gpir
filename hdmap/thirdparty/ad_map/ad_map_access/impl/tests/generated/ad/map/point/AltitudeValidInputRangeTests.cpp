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

#include "ad/map/point/AltitudeValidInputRange.hpp"

TEST(AltitudeValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::map::point::Altitude value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AltitudeValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::map::point::Altitude value(-11000);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AltitudeValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::map::point::Altitude value(9000);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(AltitudeValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::map::point::Altitude value(-11000 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(AltitudeValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::map::point::Altitude value(9000 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
