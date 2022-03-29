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

#include "ad/map/restriction/RoadUserTypeValidInputRange.hpp"

TEST(RoadUserTypeValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::restriction::RoadUserType::INVALID));
  ASSERT_TRUE(withinValidInputRange(::ad::map::restriction::RoadUserType::UNKNOWN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::restriction::RoadUserType::CAR));
  ASSERT_TRUE(withinValidInputRange(::ad::map::restriction::RoadUserType::BUS));
  ASSERT_TRUE(withinValidInputRange(::ad::map::restriction::RoadUserType::TRUCK));
  ASSERT_TRUE(withinValidInputRange(::ad::map::restriction::RoadUserType::PEDESTRIAN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::restriction::RoadUserType::MOTORBIKE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::restriction::RoadUserType::BICYCLE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::restriction::RoadUserType::CAR_ELECTRIC));
  ASSERT_TRUE(withinValidInputRange(::ad::map::restriction::RoadUserType::CAR_HYBRID));
  ASSERT_TRUE(withinValidInputRange(::ad::map::restriction::RoadUserType::CAR_PETROL));
  ASSERT_TRUE(withinValidInputRange(::ad::map::restriction::RoadUserType::CAR_DIESEL));
}

TEST(RoadUserTypeValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::INVALID));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::UNKNOWN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::BUS));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::TRUCK));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::PEDESTRIAN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::MOTORBIKE));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::BICYCLE));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR_ELECTRIC));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR_HYBRID));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR_PETROL));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR_DIESEL));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::restriction::RoadUserType>(minValue - 1)));
}
