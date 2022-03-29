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
#include "ad/map/match/MapMatchedPositionType.hpp"

TEST(MapMatchedPositionTypeTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::match::MapMatchedPositionType>("INVALID"),
            ::ad::map::match::MapMatchedPositionType::INVALID);
  ASSERT_EQ(fromString<::ad::map::match::MapMatchedPositionType>("::ad::map::match::MapMatchedPositionType::INVALID"),
            ::ad::map::match::MapMatchedPositionType::INVALID);

  ASSERT_EQ(fromString<::ad::map::match::MapMatchedPositionType>("UNKNOWN"),
            ::ad::map::match::MapMatchedPositionType::UNKNOWN);
  ASSERT_EQ(fromString<::ad::map::match::MapMatchedPositionType>("::ad::map::match::MapMatchedPositionType::UNKNOWN"),
            ::ad::map::match::MapMatchedPositionType::UNKNOWN);

  ASSERT_EQ(fromString<::ad::map::match::MapMatchedPositionType>("LANE_IN"),
            ::ad::map::match::MapMatchedPositionType::LANE_IN);
  ASSERT_EQ(fromString<::ad::map::match::MapMatchedPositionType>("::ad::map::match::MapMatchedPositionType::LANE_IN"),
            ::ad::map::match::MapMatchedPositionType::LANE_IN);

  ASSERT_EQ(fromString<::ad::map::match::MapMatchedPositionType>("LANE_LEFT"),
            ::ad::map::match::MapMatchedPositionType::LANE_LEFT);
  ASSERT_EQ(fromString<::ad::map::match::MapMatchedPositionType>("::ad::map::match::MapMatchedPositionType::LANE_LEFT"),
            ::ad::map::match::MapMatchedPositionType::LANE_LEFT);

  ASSERT_EQ(fromString<::ad::map::match::MapMatchedPositionType>("LANE_RIGHT"),
            ::ad::map::match::MapMatchedPositionType::LANE_RIGHT);
  ASSERT_EQ(
    fromString<::ad::map::match::MapMatchedPositionType>("::ad::map::match::MapMatchedPositionType::LANE_RIGHT"),
    ::ad::map::match::MapMatchedPositionType::LANE_RIGHT);

  EXPECT_ANY_THROW({ fromString<::ad::map::match::MapMatchedPositionType>("NOT A VALID ENUM LITERAL"); });
}

TEST(MapMatchedPositionTypeTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::match::MapMatchedPositionType::INVALID),
            "::ad::map::match::MapMatchedPositionType::INVALID");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::INVALID));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::INVALID));

  ASSERT_EQ(toString(::ad::map::match::MapMatchedPositionType::UNKNOWN),
            "::ad::map::match::MapMatchedPositionType::UNKNOWN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::UNKNOWN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::UNKNOWN));

  ASSERT_EQ(toString(::ad::map::match::MapMatchedPositionType::LANE_IN),
            "::ad::map::match::MapMatchedPositionType::LANE_IN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::LANE_IN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::LANE_IN));

  ASSERT_EQ(toString(::ad::map::match::MapMatchedPositionType::LANE_LEFT),
            "::ad::map::match::MapMatchedPositionType::LANE_LEFT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::LANE_LEFT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::LANE_LEFT));

  ASSERT_EQ(toString(::ad::map::match::MapMatchedPositionType::LANE_RIGHT),
            "::ad::map::match::MapMatchedPositionType::LANE_RIGHT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::LANE_RIGHT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::match::MapMatchedPositionType::LANE_RIGHT));

  ASSERT_EQ(toString(static_cast<::ad::map::match::MapMatchedPositionType>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::match::MapMatchedPositionType>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(MapMatchedPositionTypeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::match::MapMatchedPositionType value(::ad::map::match::MapMatchedPositionType::INVALID);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
