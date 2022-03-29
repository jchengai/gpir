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
#include "ad/map/route/LaneChangeDirection.hpp"

TEST(LaneChangeDirectionTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::route::LaneChangeDirection>("LeftToRight"),
            ::ad::map::route::LaneChangeDirection::LeftToRight);
  ASSERT_EQ(fromString<::ad::map::route::LaneChangeDirection>("::ad::map::route::LaneChangeDirection::LeftToRight"),
            ::ad::map::route::LaneChangeDirection::LeftToRight);

  ASSERT_EQ(fromString<::ad::map::route::LaneChangeDirection>("RightToLeft"),
            ::ad::map::route::LaneChangeDirection::RightToLeft);
  ASSERT_EQ(fromString<::ad::map::route::LaneChangeDirection>("::ad::map::route::LaneChangeDirection::RightToLeft"),
            ::ad::map::route::LaneChangeDirection::RightToLeft);

  ASSERT_EQ(fromString<::ad::map::route::LaneChangeDirection>("Invalid"),
            ::ad::map::route::LaneChangeDirection::Invalid);
  ASSERT_EQ(fromString<::ad::map::route::LaneChangeDirection>("::ad::map::route::LaneChangeDirection::Invalid"),
            ::ad::map::route::LaneChangeDirection::Invalid);

  EXPECT_ANY_THROW({ fromString<::ad::map::route::LaneChangeDirection>("NOT A VALID ENUM LITERAL"); });
}

TEST(LaneChangeDirectionTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::route::LaneChangeDirection::LeftToRight),
            "::ad::map::route::LaneChangeDirection::LeftToRight");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::LaneChangeDirection::LeftToRight));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::route::LaneChangeDirection::LeftToRight));

  ASSERT_EQ(toString(::ad::map::route::LaneChangeDirection::RightToLeft),
            "::ad::map::route::LaneChangeDirection::RightToLeft");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::LaneChangeDirection::RightToLeft));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::route::LaneChangeDirection::RightToLeft));

  ASSERT_EQ(toString(::ad::map::route::LaneChangeDirection::Invalid), "::ad::map::route::LaneChangeDirection::Invalid");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::route::LaneChangeDirection::Invalid));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::route::LaneChangeDirection::Invalid));

  ASSERT_EQ(toString(static_cast<::ad::map::route::LaneChangeDirection>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::route::LaneChangeDirection>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(LaneChangeDirectionTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::route::LaneChangeDirection value(::ad::map::route::LaneChangeDirection::LeftToRight);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
