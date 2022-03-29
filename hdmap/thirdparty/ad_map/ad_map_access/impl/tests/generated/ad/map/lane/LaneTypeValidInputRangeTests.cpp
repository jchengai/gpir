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

#include "ad/map/lane/LaneTypeValidInputRange.hpp"

TEST(LaneTypeValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneType::INVALID));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneType::UNKNOWN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneType::NORMAL));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneType::INTERSECTION));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneType::SHOULDER));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneType::EMERGENCY));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneType::MULTI));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneType::PEDESTRIAN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneType::OVERTAKING));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneType::TURN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneType::BIKE));
}

TEST(LaneTypeValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::INVALID));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::UNKNOWN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::NORMAL));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::INTERSECTION));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::SHOULDER));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::EMERGENCY));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::MULTI));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::PEDESTRIAN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::OVERTAKING));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::TURN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::BIKE));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::lane::LaneType>(minValue - 1)));
}
