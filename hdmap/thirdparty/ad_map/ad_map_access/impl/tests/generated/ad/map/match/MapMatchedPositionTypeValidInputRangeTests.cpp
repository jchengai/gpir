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

#include "ad/map/match/MapMatchedPositionTypeValidInputRange.hpp"

TEST(MapMatchedPositionTypeValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::match::MapMatchedPositionType::INVALID));
  ASSERT_TRUE(withinValidInputRange(::ad::map::match::MapMatchedPositionType::UNKNOWN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::match::MapMatchedPositionType::LANE_IN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::match::MapMatchedPositionType::LANE_LEFT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::match::MapMatchedPositionType::LANE_RIGHT));
}

TEST(MapMatchedPositionTypeValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::INVALID));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::UNKNOWN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::LANE_IN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::LANE_LEFT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::LANE_RIGHT));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::match::MapMatchedPositionType>(minValue - 1)));
}
