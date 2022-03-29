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
#include "ad/map/lane/LaneDirection.hpp"

TEST(LaneDirectionTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("INVALID"), ::ad::map::lane::LaneDirection::INVALID);
  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("::ad::map::lane::LaneDirection::INVALID"),
            ::ad::map::lane::LaneDirection::INVALID);

  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("UNKNOWN"), ::ad::map::lane::LaneDirection::UNKNOWN);
  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("::ad::map::lane::LaneDirection::UNKNOWN"),
            ::ad::map::lane::LaneDirection::UNKNOWN);

  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("POSITIVE"), ::ad::map::lane::LaneDirection::POSITIVE);
  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("::ad::map::lane::LaneDirection::POSITIVE"),
            ::ad::map::lane::LaneDirection::POSITIVE);

  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("NEGATIVE"), ::ad::map::lane::LaneDirection::NEGATIVE);
  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("::ad::map::lane::LaneDirection::NEGATIVE"),
            ::ad::map::lane::LaneDirection::NEGATIVE);

  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("REVERSABLE"), ::ad::map::lane::LaneDirection::REVERSABLE);
  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("::ad::map::lane::LaneDirection::REVERSABLE"),
            ::ad::map::lane::LaneDirection::REVERSABLE);

  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("BIDIRECTIONAL"), ::ad::map::lane::LaneDirection::BIDIRECTIONAL);
  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("::ad::map::lane::LaneDirection::BIDIRECTIONAL"),
            ::ad::map::lane::LaneDirection::BIDIRECTIONAL);

  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("NONE"), ::ad::map::lane::LaneDirection::NONE);
  ASSERT_EQ(fromString<::ad::map::lane::LaneDirection>("::ad::map::lane::LaneDirection::NONE"),
            ::ad::map::lane::LaneDirection::NONE);

  EXPECT_ANY_THROW({ fromString<::ad::map::lane::LaneDirection>("NOT A VALID ENUM LITERAL"); });
}

TEST(LaneDirectionTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::lane::LaneDirection::INVALID), "::ad::map::lane::LaneDirection::INVALID");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::INVALID));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::INVALID));

  ASSERT_EQ(toString(::ad::map::lane::LaneDirection::UNKNOWN), "::ad::map::lane::LaneDirection::UNKNOWN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::UNKNOWN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::UNKNOWN));

  ASSERT_EQ(toString(::ad::map::lane::LaneDirection::POSITIVE), "::ad::map::lane::LaneDirection::POSITIVE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::POSITIVE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::POSITIVE));

  ASSERT_EQ(toString(::ad::map::lane::LaneDirection::NEGATIVE), "::ad::map::lane::LaneDirection::NEGATIVE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::NEGATIVE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::NEGATIVE));

  ASSERT_EQ(toString(::ad::map::lane::LaneDirection::REVERSABLE), "::ad::map::lane::LaneDirection::REVERSABLE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::REVERSABLE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::REVERSABLE));

  ASSERT_EQ(toString(::ad::map::lane::LaneDirection::BIDIRECTIONAL), "::ad::map::lane::LaneDirection::BIDIRECTIONAL");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::BIDIRECTIONAL));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::BIDIRECTIONAL));

  ASSERT_EQ(toString(::ad::map::lane::LaneDirection::NONE), "::ad::map::lane::LaneDirection::NONE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::NONE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::LaneDirection::NONE));

  ASSERT_EQ(toString(static_cast<::ad::map::lane::LaneDirection>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::lane::LaneDirection>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(LaneDirectionTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::lane::LaneDirection value(::ad::map::lane::LaneDirection::INVALID);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
