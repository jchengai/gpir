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
#include "ad/map/access/TrafficType.hpp"

TEST(TrafficTypeTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::access::TrafficType>("INVALID"), ::ad::map::access::TrafficType::INVALID);
  ASSERT_EQ(fromString<::ad::map::access::TrafficType>("::ad::map::access::TrafficType::INVALID"),
            ::ad::map::access::TrafficType::INVALID);

  ASSERT_EQ(fromString<::ad::map::access::TrafficType>("LEFT_HAND_TRAFFIC"),
            ::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC);
  ASSERT_EQ(fromString<::ad::map::access::TrafficType>("::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC"),
            ::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC);

  ASSERT_EQ(fromString<::ad::map::access::TrafficType>("RIGHT_HAND_TRAFFIC"),
            ::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC);
  ASSERT_EQ(fromString<::ad::map::access::TrafficType>("::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC"),
            ::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC);

  EXPECT_ANY_THROW({ fromString<::ad::map::access::TrafficType>("NOT A VALID ENUM LITERAL"); });
}

TEST(TrafficTypeTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::access::TrafficType::INVALID), "::ad::map::access::TrafficType::INVALID");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::access::TrafficType::INVALID));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::access::TrafficType::INVALID));

  ASSERT_EQ(toString(::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC),
            "::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC));

  ASSERT_EQ(toString(::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC),
            "::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC));

  ASSERT_EQ(toString(static_cast<::ad::map::access::TrafficType>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::access::TrafficType>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(TrafficTypeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::access::TrafficType value(::ad::map::access::TrafficType::INVALID);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
