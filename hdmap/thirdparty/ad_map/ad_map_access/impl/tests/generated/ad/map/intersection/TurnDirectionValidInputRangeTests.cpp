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

#include "ad/map/intersection/TurnDirectionValidInputRange.hpp"

TEST(TurnDirectionValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::TurnDirection::Unknown));
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::TurnDirection::Right));
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::TurnDirection::Straight));
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::TurnDirection::Left));
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::TurnDirection::UTurn));
}

TEST(TurnDirectionValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::Unknown));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::Right));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::Straight));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::Left));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::UTurn));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::intersection::TurnDirection>(minValue - 1)));
}
