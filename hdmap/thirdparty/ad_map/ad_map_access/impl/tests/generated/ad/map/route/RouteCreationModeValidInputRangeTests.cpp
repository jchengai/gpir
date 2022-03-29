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

#include "ad/map/route/RouteCreationModeValidInputRange.hpp"

TEST(RouteCreationModeValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::route::RouteCreationMode::Undefined));
  ASSERT_TRUE(withinValidInputRange(::ad::map::route::RouteCreationMode::SameDrivingDirection));
  ASSERT_TRUE(withinValidInputRange(::ad::map::route::RouteCreationMode::AllRoutableLanes));
  ASSERT_TRUE(withinValidInputRange(::ad::map::route::RouteCreationMode::AllNeighborLanes));
}

TEST(RouteCreationModeValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::RouteCreationMode::Undefined));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::RouteCreationMode::SameDrivingDirection));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::RouteCreationMode::AllRoutableLanes));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::RouteCreationMode::AllNeighborLanes));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::route::RouteCreationMode>(minValue - 1)));
}
