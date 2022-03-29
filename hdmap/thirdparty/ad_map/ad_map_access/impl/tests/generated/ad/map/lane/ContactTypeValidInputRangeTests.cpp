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

#include "ad/map/lane/ContactTypeValidInputRange.hpp"

TEST(ContactTypeValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::INVALID));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::UNKNOWN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::FREE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::LANE_CHANGE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::LANE_CONTINUATION));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::LANE_END));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::SINGLE_POINT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::STOP));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::STOP_ALL));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::YIELD));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::GATE_BARRIER));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::GATE_TOLBOOTH));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::GATE_SPIKES));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::GATE_SPIKES_CONTRA));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::CURB_UP));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::CURB_DOWN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::SPEED_BUMP));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::TRAFFIC_LIGHT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::CROSSWALK));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::PRIO_TO_RIGHT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::RIGHT_OF_WAY));
  ASSERT_TRUE(withinValidInputRange(::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT));
}

TEST(ContactTypeValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::INVALID));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::UNKNOWN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::FREE));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::LANE_CHANGE));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::LANE_CONTINUATION));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::LANE_END));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::SINGLE_POINT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::STOP));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::STOP_ALL));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::YIELD));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::GATE_BARRIER));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::GATE_TOLBOOTH));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::GATE_SPIKES));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::GATE_SPIKES_CONTRA));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::CURB_UP));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::CURB_DOWN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::SPEED_BUMP));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::TRAFFIC_LIGHT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::CROSSWALK));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::PRIO_TO_RIGHT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::RIGHT_OF_WAY));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::lane::ContactType>(minValue - 1)));
}
