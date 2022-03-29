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

#include "ad/map/point/ECEFCoordinateValidInputRange.hpp"

TEST(ECEFCoordinateValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::map::point::ECEFCoordinate value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFCoordinateValidInputRangeTests, testValidInputRangeMinOk)
{
  ::ad::map::point::ECEFCoordinate value(-1e9);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFCoordinateValidInputRangeTests, testValidInputRangeMaxOk)
{
  ::ad::map::point::ECEFCoordinate value(1e9);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFCoordinateValidInputRangeTests, testValidInputRangeBelowMin)
{
  ::ad::map::point::ECEFCoordinate value(::ad::map::point::ECEFCoordinate::cMinValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFCoordinateValidInputRangeTests, testValidInputRangeExceedsMax)
{
  ::ad::map::point::ECEFCoordinate value(::ad::map::point::ECEFCoordinate::cMaxValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFCoordinateValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::map::point::ECEFCoordinate value(-6400000);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ECEFCoordinateValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::map::point::ECEFCoordinate value(6400000);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ECEFCoordinateValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::map::point::ECEFCoordinate value(-6400000 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ECEFCoordinateValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::map::point::ECEFCoordinate value(6400000 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
