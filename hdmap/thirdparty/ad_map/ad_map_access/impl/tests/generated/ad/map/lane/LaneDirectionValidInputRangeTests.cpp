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

#include "ad/map/lane/LaneDirectionValidInputRange.hpp"

TEST(LaneDirectionValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneDirection::INVALID));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneDirection::UNKNOWN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneDirection::POSITIVE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneDirection::NEGATIVE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneDirection::REVERSABLE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneDirection::BIDIRECTIONAL));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::LaneDirection::NONE));
}

TEST(LaneDirectionValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::INVALID));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::UNKNOWN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::POSITIVE));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::NEGATIVE));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::REVERSABLE));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::BIDIRECTIONAL));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::NONE));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::lane::LaneDirection>(minValue - 1)));
}
