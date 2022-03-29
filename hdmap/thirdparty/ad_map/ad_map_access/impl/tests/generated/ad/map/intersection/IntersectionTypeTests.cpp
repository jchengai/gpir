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
#include "ad/map/intersection/IntersectionType.hpp"

TEST(IntersectionTypeTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>("Unknown"),
            ::ad::map::intersection::IntersectionType::Unknown);
  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>("::ad::map::intersection::IntersectionType::Unknown"),
            ::ad::map::intersection::IntersectionType::Unknown);

  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>("Yield"),
            ::ad::map::intersection::IntersectionType::Yield);
  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>("::ad::map::intersection::IntersectionType::Yield"),
            ::ad::map::intersection::IntersectionType::Yield);

  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>("Stop"),
            ::ad::map::intersection::IntersectionType::Stop);
  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>("::ad::map::intersection::IntersectionType::Stop"),
            ::ad::map::intersection::IntersectionType::Stop);

  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>("AllWayStop"),
            ::ad::map::intersection::IntersectionType::AllWayStop);
  ASSERT_EQ(
    fromString<::ad::map::intersection::IntersectionType>("::ad::map::intersection::IntersectionType::AllWayStop"),
    ::ad::map::intersection::IntersectionType::AllWayStop);

  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>("HasWay"),
            ::ad::map::intersection::IntersectionType::HasWay);
  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>("::ad::map::intersection::IntersectionType::HasWay"),
            ::ad::map::intersection::IntersectionType::HasWay);

  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>("Crosswalk"),
            ::ad::map::intersection::IntersectionType::Crosswalk);
  ASSERT_EQ(
    fromString<::ad::map::intersection::IntersectionType>("::ad::map::intersection::IntersectionType::Crosswalk"),
    ::ad::map::intersection::IntersectionType::Crosswalk);

  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>("PriorityToRight"),
            ::ad::map::intersection::IntersectionType::PriorityToRight);
  ASSERT_EQ(
    fromString<::ad::map::intersection::IntersectionType>("::ad::map::intersection::IntersectionType::PriorityToRight"),
    ::ad::map::intersection::IntersectionType::PriorityToRight);

  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>("PriorityToRightAndStraight"),
            ::ad::map::intersection::IntersectionType::PriorityToRightAndStraight);
  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>(
              "::ad::map::intersection::IntersectionType::PriorityToRightAndStraight"),
            ::ad::map::intersection::IntersectionType::PriorityToRightAndStraight);

  ASSERT_EQ(fromString<::ad::map::intersection::IntersectionType>("TrafficLight"),
            ::ad::map::intersection::IntersectionType::TrafficLight);
  ASSERT_EQ(
    fromString<::ad::map::intersection::IntersectionType>("::ad::map::intersection::IntersectionType::TrafficLight"),
    ::ad::map::intersection::IntersectionType::TrafficLight);

  EXPECT_ANY_THROW({ fromString<::ad::map::intersection::IntersectionType>("NOT A VALID ENUM LITERAL"); });
}

TEST(IntersectionTypeTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::intersection::IntersectionType::Unknown),
            "::ad::map::intersection::IntersectionType::Unknown");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::Unknown));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::Unknown));

  ASSERT_EQ(toString(::ad::map::intersection::IntersectionType::Yield),
            "::ad::map::intersection::IntersectionType::Yield");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::Yield));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::Yield));

  ASSERT_EQ(toString(::ad::map::intersection::IntersectionType::Stop),
            "::ad::map::intersection::IntersectionType::Stop");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::Stop));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::Stop));

  ASSERT_EQ(toString(::ad::map::intersection::IntersectionType::AllWayStop),
            "::ad::map::intersection::IntersectionType::AllWayStop");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::AllWayStop));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::AllWayStop));

  ASSERT_EQ(toString(::ad::map::intersection::IntersectionType::HasWay),
            "::ad::map::intersection::IntersectionType::HasWay");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::HasWay));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::HasWay));

  ASSERT_EQ(toString(::ad::map::intersection::IntersectionType::Crosswalk),
            "::ad::map::intersection::IntersectionType::Crosswalk");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::Crosswalk));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::Crosswalk));

  ASSERT_EQ(toString(::ad::map::intersection::IntersectionType::PriorityToRight),
            "::ad::map::intersection::IntersectionType::PriorityToRight");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::PriorityToRight));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::PriorityToRight));

  ASSERT_EQ(toString(::ad::map::intersection::IntersectionType::PriorityToRightAndStraight),
            "::ad::map::intersection::IntersectionType::PriorityToRightAndStraight");
  minValue
    = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::PriorityToRightAndStraight));
  maxValue
    = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::PriorityToRightAndStraight));

  ASSERT_EQ(toString(::ad::map::intersection::IntersectionType::TrafficLight),
            "::ad::map::intersection::IntersectionType::TrafficLight");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::TrafficLight));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::IntersectionType::TrafficLight));

  ASSERT_EQ(toString(static_cast<::ad::map::intersection::IntersectionType>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::intersection::IntersectionType>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(IntersectionTypeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::intersection::IntersectionType value(::ad::map::intersection::IntersectionType::Unknown);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
