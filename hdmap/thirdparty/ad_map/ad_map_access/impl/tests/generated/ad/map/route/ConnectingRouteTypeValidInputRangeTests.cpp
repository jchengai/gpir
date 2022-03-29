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

#include "ad/map/route/ConnectingRouteTypeValidInputRange.hpp"

TEST(ConnectingRouteTypeValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::route::ConnectingRouteType::Invalid));
  ASSERT_TRUE(withinValidInputRange(::ad::map::route::ConnectingRouteType::Following));
  ASSERT_TRUE(withinValidInputRange(::ad::map::route::ConnectingRouteType::Opposing));
  ASSERT_TRUE(withinValidInputRange(::ad::map::route::ConnectingRouteType::Merging));
}

TEST(ConnectingRouteTypeValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::ConnectingRouteType::Invalid));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::ConnectingRouteType::Following));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::ConnectingRouteType::Opposing));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::ConnectingRouteType::Merging));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::route::ConnectingRouteType>(minValue - 1)));
}
