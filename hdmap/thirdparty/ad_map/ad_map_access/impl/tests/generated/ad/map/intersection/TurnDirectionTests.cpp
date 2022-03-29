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
#include "ad/map/intersection/TurnDirection.hpp"

TEST(TurnDirectionTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::intersection::TurnDirection>("Unknown"),
            ::ad::map::intersection::TurnDirection::Unknown);
  ASSERT_EQ(fromString<::ad::map::intersection::TurnDirection>("::ad::map::intersection::TurnDirection::Unknown"),
            ::ad::map::intersection::TurnDirection::Unknown);

  ASSERT_EQ(fromString<::ad::map::intersection::TurnDirection>("Right"), ::ad::map::intersection::TurnDirection::Right);
  ASSERT_EQ(fromString<::ad::map::intersection::TurnDirection>("::ad::map::intersection::TurnDirection::Right"),
            ::ad::map::intersection::TurnDirection::Right);

  ASSERT_EQ(fromString<::ad::map::intersection::TurnDirection>("Straight"),
            ::ad::map::intersection::TurnDirection::Straight);
  ASSERT_EQ(fromString<::ad::map::intersection::TurnDirection>("::ad::map::intersection::TurnDirection::Straight"),
            ::ad::map::intersection::TurnDirection::Straight);

  ASSERT_EQ(fromString<::ad::map::intersection::TurnDirection>("Left"), ::ad::map::intersection::TurnDirection::Left);
  ASSERT_EQ(fromString<::ad::map::intersection::TurnDirection>("::ad::map::intersection::TurnDirection::Left"),
            ::ad::map::intersection::TurnDirection::Left);

  ASSERT_EQ(fromString<::ad::map::intersection::TurnDirection>("UTurn"), ::ad::map::intersection::TurnDirection::UTurn);
  ASSERT_EQ(fromString<::ad::map::intersection::TurnDirection>("::ad::map::intersection::TurnDirection::UTurn"),
            ::ad::map::intersection::TurnDirection::UTurn);

  EXPECT_ANY_THROW({ fromString<::ad::map::intersection::TurnDirection>("NOT A VALID ENUM LITERAL"); });
}

TEST(TurnDirectionTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::intersection::TurnDirection::Unknown),
            "::ad::map::intersection::TurnDirection::Unknown");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::Unknown));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::Unknown));

  ASSERT_EQ(toString(::ad::map::intersection::TurnDirection::Right), "::ad::map::intersection::TurnDirection::Right");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::Right));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::Right));

  ASSERT_EQ(toString(::ad::map::intersection::TurnDirection::Straight),
            "::ad::map::intersection::TurnDirection::Straight");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::Straight));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::Straight));

  ASSERT_EQ(toString(::ad::map::intersection::TurnDirection::Left), "::ad::map::intersection::TurnDirection::Left");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::Left));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::Left));

  ASSERT_EQ(toString(::ad::map::intersection::TurnDirection::UTurn), "::ad::map::intersection::TurnDirection::UTurn");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::UTurn));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::intersection::TurnDirection::UTurn));

  ASSERT_EQ(toString(static_cast<::ad::map::intersection::TurnDirection>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::intersection::TurnDirection>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(TurnDirectionTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::intersection::TurnDirection value(::ad::map::intersection::TurnDirection::Unknown);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
