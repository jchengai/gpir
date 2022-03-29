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

#include "ad/map/landmark/TrafficLightTypeValidInputRange.hpp"

TEST(TrafficLightTypeValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::INVALID));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::UNKNOWN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN));
}

TEST(TrafficLightTypeValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::INVALID));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::UNKNOWN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN));
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::landmark::TrafficLightType>(minValue - 1)));
}
