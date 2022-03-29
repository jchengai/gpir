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
#include "ad/map/landmark/TrafficLightType.hpp"

TEST(TrafficLightTypeTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("INVALID"),
            ::ad::map::landmark::TrafficLightType::INVALID);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("::ad::map::landmark::TrafficLightType::INVALID"),
            ::ad::map::landmark::TrafficLightType::INVALID);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("UNKNOWN"),
            ::ad::map::landmark::TrafficLightType::UNKNOWN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("::ad::map::landmark::TrafficLightType::UNKNOWN"),
            ::ad::map::landmark::TrafficLightType::UNKNOWN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("SOLID_RED_YELLOW"),
            ::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficLightType>("::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW"),
    ::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("SOLID_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficLightType>("::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN"),
    ::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("LEFT_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficLightType>("::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN"),
    ::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("RIGHT_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficLightType>("::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN"),
    ::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("STRAIGHT_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>(
              "::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("LEFT_STRAIGHT_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>(
              "::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("RIGHT_STRAIGHT_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>(
              "::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("PEDESTRIAN_RED_GREEN"),
            ::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficLightType>("::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN"),
    ::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("BIKE_RED_GREEN"),
            ::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN"),
            ::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("BIKE_PEDESTRIAN_RED_GREEN"),
            ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>(
              "::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN"),
            ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("PEDESTRIAN_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>(
              "::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("BIKE_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN);
  ASSERT_EQ(
    fromString<::ad::map::landmark::TrafficLightType>("::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN"),
    ::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN);

  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>("BIKE_PEDESTRIAN_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN);
  ASSERT_EQ(fromString<::ad::map::landmark::TrafficLightType>(
              "::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN"),
            ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN);

  EXPECT_ANY_THROW({ fromString<::ad::map::landmark::TrafficLightType>("NOT A VALID ENUM LITERAL"); });
}

TEST(TrafficLightTypeTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::INVALID), "::ad::map::landmark::TrafficLightType::INVALID");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::INVALID));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::INVALID));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::UNKNOWN), "::ad::map::landmark::TrafficLightType::UNKNOWN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::UNKNOWN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::UNKNOWN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW),
            "::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN),
            "::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN),
            "::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN),
            "::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN),
            "::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN),
            "::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN),
            "::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN),
            "::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN),
            "::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN),
            "::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN),
            "::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN),
            "::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN));

  ASSERT_EQ(toString(::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN),
            "::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN));

  ASSERT_EQ(toString(static_cast<::ad::map::landmark::TrafficLightType>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::landmark::TrafficLightType>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(TrafficLightTypeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::landmark::TrafficLightType value(::ad::map::landmark::TrafficLightType::INVALID);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
