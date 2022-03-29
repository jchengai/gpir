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
#include "ad/map/route/ConnectingRouteType.hpp"

TEST(ConnectingRouteTypeTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::route::ConnectingRouteType>("Invalid"),
            ::ad::map::route::ConnectingRouteType::Invalid);
  ASSERT_EQ(fromString<::ad::map::route::ConnectingRouteType>("::ad::map::route::ConnectingRouteType::Invalid"),
            ::ad::map::route::ConnectingRouteType::Invalid);

  ASSERT_EQ(fromString<::ad::map::route::ConnectingRouteType>("Following"),
            ::ad::map::route::ConnectingRouteType::Following);
  ASSERT_EQ(fromString<::ad::map::route::ConnectingRouteType>("::ad::map::route::ConnectingRouteType::Following"),
            ::ad::map::route::ConnectingRouteType::Following);

  ASSERT_EQ(fromString<::ad::map::route::ConnectingRouteType>("Opposing"),
            ::ad::map::route::ConnectingRouteType::Opposing);
  ASSERT_EQ(fromString<::ad::map::route::ConnectingRouteType>("::ad::map::route::ConnectingRouteType::Opposing"),
            ::ad::map::route::ConnectingRouteType::Opposing);

  ASSERT_EQ(fromString<::ad::map::route::ConnectingRouteType>("Merging"),
            ::ad::map::route::ConnectingRouteType::Merging);
  ASSERT_EQ(fromString<::ad::map::route::ConnectingRouteType>("::ad::map::route::ConnectingRouteType::Merging"),
            ::ad::map::route::ConnectingRouteType::Merging);

  EXPECT_ANY_THROW({ fromString<::ad::map::route::ConnectingRouteType>("NOT A VALID ENUM LITERAL"); });
}

TEST(ConnectingRouteTypeTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::route::ConnectingRouteType::Invalid), "::ad::map::route::ConnectingRouteType::Invalid");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::ConnectingRouteType::Invalid));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::route::ConnectingRouteType::Invalid));

  ASSERT_EQ(toString(::ad::map::route::ConnectingRouteType::Following),
            "::ad::map::route::ConnectingRouteType::Following");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::ConnectingRouteType::Following));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::route::ConnectingRouteType::Following));

  ASSERT_EQ(toString(::ad::map::route::ConnectingRouteType::Opposing),
            "::ad::map::route::ConnectingRouteType::Opposing");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::ConnectingRouteType::Opposing));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::route::ConnectingRouteType::Opposing));

  ASSERT_EQ(toString(::ad::map::route::ConnectingRouteType::Merging), "::ad::map::route::ConnectingRouteType::Merging");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::ConnectingRouteType::Merging));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::route::ConnectingRouteType::Merging));

  ASSERT_EQ(toString(static_cast<::ad::map::route::ConnectingRouteType>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::route::ConnectingRouteType>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(ConnectingRouteTypeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::route::ConnectingRouteType value(::ad::map::route::ConnectingRouteType::Invalid);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
