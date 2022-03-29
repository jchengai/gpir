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

#include "ad/map/point/ENUCoordinateValidInputRange.hpp"

TEST(ENUCoordinateValidInputRangeTests, testValidInputRangeUninitialized)
{
  ::ad::map::point::ENUCoordinate value;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUCoordinateValidInputRangeTests, testValidInputRangeMinOk)
{
  ::ad::map::point::ENUCoordinate value(-1e6);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUCoordinateValidInputRangeTests, testValidInputRangeMaxOk)
{
  ::ad::map::point::ENUCoordinate value(1e6);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUCoordinateValidInputRangeTests, testValidInputRangeBelowMin)
{
  ::ad::map::point::ENUCoordinate value(::ad::map::point::ENUCoordinate::cMinValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUCoordinateValidInputRangeTests, testValidInputRangeExceedsMax)
{
  ::ad::map::point::ENUCoordinate value(::ad::map::point::ENUCoordinate::cMaxValue * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUCoordinateValidInputRangeTests, testValidInputRangeInputMinOk)
{
  ::ad::map::point::ENUCoordinate value(-16384);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ENUCoordinateValidInputRangeTests, testValidInputRangeInputMaxOk)
{
  ::ad::map::point::ENUCoordinate value(16384);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ENUCoordinateValidInputRangeTests, testValidInputRangeBelowInputMin)
{
  ::ad::map::point::ENUCoordinate value(-16384 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ENUCoordinateValidInputRangeTests, testValidInputRangeExceedsInputMax)
{
  ::ad::map::point::ENUCoordinate value(16384 * 1.1);
  ASSERT_FALSE(withinValidInputRange(value));
}
