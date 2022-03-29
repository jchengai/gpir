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

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wself-assign-overloaded"
#endif

#include <gtest/gtest.h>
#include <limits>
#include "ad/map/route/RouteCreationMode.hpp"

TEST(RouteCreationModeTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::route::RouteCreationMode>("Undefined"),
            ::ad::map::route::RouteCreationMode::Undefined);
  ASSERT_EQ(fromString<::ad::map::route::RouteCreationMode>("::ad::map::route::RouteCreationMode::Undefined"),
            ::ad::map::route::RouteCreationMode::Undefined);

  ASSERT_EQ(fromString<::ad::map::route::RouteCreationMode>("SameDrivingDirection"),
            ::ad::map::route::RouteCreationMode::SameDrivingDirection);
  ASSERT_EQ(
    fromString<::ad::map::route::RouteCreationMode>("::ad::map::route::RouteCreationMode::SameDrivingDirection"),
    ::ad::map::route::RouteCreationMode::SameDrivingDirection);

  ASSERT_EQ(fromString<::ad::map::route::RouteCreationMode>("AllRoutableLanes"),
            ::ad::map::route::RouteCreationMode::AllRoutableLanes);
  ASSERT_EQ(fromString<::ad::map::route::RouteCreationMode>("::ad::map::route::RouteCreationMode::AllRoutableLanes"),
            ::ad::map::route::RouteCreationMode::AllRoutableLanes);

  ASSERT_EQ(fromString<::ad::map::route::RouteCreationMode>("AllNeighborLanes"),
            ::ad::map::route::RouteCreationMode::AllNeighborLanes);
  ASSERT_EQ(fromString<::ad::map::route::RouteCreationMode>("::ad::map::route::RouteCreationMode::AllNeighborLanes"),
            ::ad::map::route::RouteCreationMode::AllNeighborLanes);

  EXPECT_ANY_THROW({ fromString<::ad::map::route::RouteCreationMode>("NOT A VALID ENUM LITERAL"); });
}

TEST(RouteCreationModeTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::route::RouteCreationMode::Undefined), "::ad::map::route::RouteCreationMode::Undefined");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::RouteCreationMode::Undefined));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::route::RouteCreationMode::Undefined));

  ASSERT_EQ(toString(::ad::map::route::RouteCreationMode::SameDrivingDirection),
            "::ad::map::route::RouteCreationMode::SameDrivingDirection");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::RouteCreationMode::SameDrivingDirection));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::route::RouteCreationMode::SameDrivingDirection));

  ASSERT_EQ(toString(::ad::map::route::RouteCreationMode::AllRoutableLanes),
            "::ad::map::route::RouteCreationMode::AllRoutableLanes");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::RouteCreationMode::AllRoutableLanes));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::route::RouteCreationMode::AllRoutableLanes));

  ASSERT_EQ(toString(::ad::map::route::RouteCreationMode::AllNeighborLanes),
            "::ad::map::route::RouteCreationMode::AllNeighborLanes");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::RouteCreationMode::AllNeighborLanes));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::route::RouteCreationMode::AllNeighborLanes));

  ASSERT_EQ(toString(static_cast<::ad::map::route::RouteCreationMode>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::route::RouteCreationMode>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(RouteCreationModeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::route::RouteCreationMode value(::ad::map::route::RouteCreationMode::Undefined);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
