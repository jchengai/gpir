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

#include "ad/map/route/LaneChangeDirectionValidInputRange.hpp"

TEST(LaneChangeDirectionValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::route::LaneChangeDirection::LeftToRight));
  ASSERT_TRUE(withinValidInputRange(::ad::map::route::LaneChangeDirection::RightToLeft));
  ASSERT_TRUE(withinValidInputRange(::ad::map::route::LaneChangeDirection::Invalid));
}

TEST(LaneChangeDirectionValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::LaneChangeDirection::LeftToRight));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::LaneChangeDirection::RightToLeft));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::LaneChangeDirection::Invalid));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::route::LaneChangeDirection>(minValue - 1)));
}
