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

#include "ad/map/access/TrafficTypeValidInputRange.hpp"

TEST(TrafficTypeValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::access::TrafficType::INVALID));
  ASSERT_TRUE(withinValidInputRange(::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC));
  ASSERT_TRUE(withinValidInputRange(::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC));
}

TEST(TrafficTypeValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::access::TrafficType::INVALID));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::access::TrafficType>(minValue - 1)));
}
