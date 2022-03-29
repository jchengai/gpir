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

#include "ad/map/landmark/LandmarkTypeValidInputRange.hpp"

TEST(LandmarkTypeValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::INVALID));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::UNKNOWN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::TRAFFIC_SIGN));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::POLE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::GUIDE_POST));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::TREE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::STREET_LAMP));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::POSTBOX));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::MANHOLE));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::POWERCABINET));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::FIRE_HYDRANT));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::BOLLARD));
  ASSERT_TRUE(withinValidInputRange(::ad::map::landmark::LandmarkType::OTHER));
}

TEST(LandmarkTypeValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::INVALID));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::UNKNOWN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::TRAFFIC_SIGN));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::POLE));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::GUIDE_POST));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::TREE));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::STREET_LAMP));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::POSTBOX));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::MANHOLE));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::POWERCABINET));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::FIRE_HYDRANT));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::BOLLARD));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::OTHER));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::landmark::LandmarkType>(minValue - 1)));
}
