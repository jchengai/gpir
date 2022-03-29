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

#include "ad/map/intersection/IntersectionTypeValidInputRange.hpp"

TEST(IntersectionTypeValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::IntersectionType::Unknown));
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::IntersectionType::Yield));
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::IntersectionType::Stop));
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::IntersectionType::AllWayStop));
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::IntersectionType::HasWay));
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::IntersectionType::Crosswalk));
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::IntersectionType::PriorityToRight));
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::IntersectionType::PriorityToRightAndStraight));
  ASSERT_TRUE(withinValidInputRange(::ad::map::intersection::IntersectionType::TrafficLight));
}

TEST(IntersectionTypeValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::Unknown));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::Yield));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::Stop));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::AllWayStop));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::HasWay));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::Crosswalk));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::PriorityToRight));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::PriorityToRightAndStraight));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::TrafficLight));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::intersection::IntersectionType>(minValue - 1)));
}
