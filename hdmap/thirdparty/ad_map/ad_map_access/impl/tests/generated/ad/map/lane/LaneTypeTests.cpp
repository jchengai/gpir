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
#include "ad/map/lane/LaneType.hpp"

TEST(LaneTypeTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("INVALID"), ::ad::map::lane::LaneType::INVALID);
  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("::ad::map::lane::LaneType::INVALID"),
            ::ad::map::lane::LaneType::INVALID);

  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("UNKNOWN"), ::ad::map::lane::LaneType::UNKNOWN);
  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("::ad::map::lane::LaneType::UNKNOWN"),
            ::ad::map::lane::LaneType::UNKNOWN);

  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("NORMAL"), ::ad::map::lane::LaneType::NORMAL);
  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("::ad::map::lane::LaneType::NORMAL"),
            ::ad::map::lane::LaneType::NORMAL);

  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("INTERSECTION"), ::ad::map::lane::LaneType::INTERSECTION);
  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("::ad::map::lane::LaneType::INTERSECTION"),
            ::ad::map::lane::LaneType::INTERSECTION);

  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("SHOULDER"), ::ad::map::lane::LaneType::SHOULDER);
  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("::ad::map::lane::LaneType::SHOULDER"),
            ::ad::map::lane::LaneType::SHOULDER);

  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("EMERGENCY"), ::ad::map::lane::LaneType::EMERGENCY);
  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("::ad::map::lane::LaneType::EMERGENCY"),
            ::ad::map::lane::LaneType::EMERGENCY);

  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("MULTI"), ::ad::map::lane::LaneType::MULTI);
  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("::ad::map::lane::LaneType::MULTI"),
            ::ad::map::lane::LaneType::MULTI);

  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("PEDESTRIAN"), ::ad::map::lane::LaneType::PEDESTRIAN);
  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("::ad::map::lane::LaneType::PEDESTRIAN"),
            ::ad::map::lane::LaneType::PEDESTRIAN);

  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("OVERTAKING"), ::ad::map::lane::LaneType::OVERTAKING);
  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("::ad::map::lane::LaneType::OVERTAKING"),
            ::ad::map::lane::LaneType::OVERTAKING);

  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("TURN"), ::ad::map::lane::LaneType::TURN);
  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("::ad::map::lane::LaneType::TURN"), ::ad::map::lane::LaneType::TURN);

  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("BIKE"), ::ad::map::lane::LaneType::BIKE);
  ASSERT_EQ(fromString<::ad::map::lane::LaneType>("::ad::map::lane::LaneType::BIKE"), ::ad::map::lane::LaneType::BIKE);

  EXPECT_ANY_THROW({ fromString<::ad::map::lane::LaneType>("NOT A VALID ENUM LITERAL"); });
}

TEST(LaneTypeTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::lane::LaneType::INVALID), "::ad::map::lane::LaneType::INVALID");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::INVALID));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneType::INVALID));

  ASSERT_EQ(toString(::ad::map::lane::LaneType::UNKNOWN), "::ad::map::lane::LaneType::UNKNOWN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::UNKNOWN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneType::UNKNOWN));

  ASSERT_EQ(toString(::ad::map::lane::LaneType::NORMAL), "::ad::map::lane::LaneType::NORMAL");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::NORMAL));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneType::NORMAL));

  ASSERT_EQ(toString(::ad::map::lane::LaneType::INTERSECTION), "::ad::map::lane::LaneType::INTERSECTION");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::INTERSECTION));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneType::INTERSECTION));

  ASSERT_EQ(toString(::ad::map::lane::LaneType::SHOULDER), "::ad::map::lane::LaneType::SHOULDER");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::SHOULDER));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneType::SHOULDER));

  ASSERT_EQ(toString(::ad::map::lane::LaneType::EMERGENCY), "::ad::map::lane::LaneType::EMERGENCY");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::EMERGENCY));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneType::EMERGENCY));

  ASSERT_EQ(toString(::ad::map::lane::LaneType::MULTI), "::ad::map::lane::LaneType::MULTI");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::MULTI));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneType::MULTI));

  ASSERT_EQ(toString(::ad::map::lane::LaneType::PEDESTRIAN), "::ad::map::lane::LaneType::PEDESTRIAN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::PEDESTRIAN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneType::PEDESTRIAN));

  ASSERT_EQ(toString(::ad::map::lane::LaneType::OVERTAKING), "::ad::map::lane::LaneType::OVERTAKING");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::OVERTAKING));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneType::OVERTAKING));

  ASSERT_EQ(toString(::ad::map::lane::LaneType::TURN), "::ad::map::lane::LaneType::TURN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::TURN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneType::TURN));

  ASSERT_EQ(toString(::ad::map::lane::LaneType::BIKE), "::ad::map::lane::LaneType::BIKE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneType::BIKE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneType::BIKE));

  ASSERT_EQ(toString(static_cast<::ad::map::lane::LaneType>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::lane::LaneType>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(LaneTypeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::lane::LaneType value(::ad::map::lane::LaneType::INVALID);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
