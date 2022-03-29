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
#include "ad/map/lane/ContactType.hpp"

TEST(ContactTypeTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("INVALID"), ::ad::map::lane::ContactType::INVALID);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::INVALID"),
            ::ad::map::lane::ContactType::INVALID);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("UNKNOWN"), ::ad::map::lane::ContactType::UNKNOWN);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::UNKNOWN"),
            ::ad::map::lane::ContactType::UNKNOWN);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("FREE"), ::ad::map::lane::ContactType::FREE);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::FREE"),
            ::ad::map::lane::ContactType::FREE);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("LANE_CHANGE"), ::ad::map::lane::ContactType::LANE_CHANGE);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::LANE_CHANGE"),
            ::ad::map::lane::ContactType::LANE_CHANGE);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("LANE_CONTINUATION"),
            ::ad::map::lane::ContactType::LANE_CONTINUATION);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::LANE_CONTINUATION"),
            ::ad::map::lane::ContactType::LANE_CONTINUATION);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("LANE_END"), ::ad::map::lane::ContactType::LANE_END);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::LANE_END"),
            ::ad::map::lane::ContactType::LANE_END);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("SINGLE_POINT"), ::ad::map::lane::ContactType::SINGLE_POINT);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::SINGLE_POINT"),
            ::ad::map::lane::ContactType::SINGLE_POINT);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("STOP"), ::ad::map::lane::ContactType::STOP);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::STOP"),
            ::ad::map::lane::ContactType::STOP);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("STOP_ALL"), ::ad::map::lane::ContactType::STOP_ALL);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::STOP_ALL"),
            ::ad::map::lane::ContactType::STOP_ALL);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("YIELD"), ::ad::map::lane::ContactType::YIELD);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::YIELD"),
            ::ad::map::lane::ContactType::YIELD);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("GATE_BARRIER"), ::ad::map::lane::ContactType::GATE_BARRIER);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::GATE_BARRIER"),
            ::ad::map::lane::ContactType::GATE_BARRIER);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("GATE_TOLBOOTH"), ::ad::map::lane::ContactType::GATE_TOLBOOTH);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::GATE_TOLBOOTH"),
            ::ad::map::lane::ContactType::GATE_TOLBOOTH);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("GATE_SPIKES"), ::ad::map::lane::ContactType::GATE_SPIKES);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::GATE_SPIKES"),
            ::ad::map::lane::ContactType::GATE_SPIKES);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("GATE_SPIKES_CONTRA"),
            ::ad::map::lane::ContactType::GATE_SPIKES_CONTRA);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::GATE_SPIKES_CONTRA"),
            ::ad::map::lane::ContactType::GATE_SPIKES_CONTRA);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("CURB_UP"), ::ad::map::lane::ContactType::CURB_UP);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::CURB_UP"),
            ::ad::map::lane::ContactType::CURB_UP);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("CURB_DOWN"), ::ad::map::lane::ContactType::CURB_DOWN);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::CURB_DOWN"),
            ::ad::map::lane::ContactType::CURB_DOWN);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("SPEED_BUMP"), ::ad::map::lane::ContactType::SPEED_BUMP);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::SPEED_BUMP"),
            ::ad::map::lane::ContactType::SPEED_BUMP);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("TRAFFIC_LIGHT"), ::ad::map::lane::ContactType::TRAFFIC_LIGHT);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::TRAFFIC_LIGHT"),
            ::ad::map::lane::ContactType::TRAFFIC_LIGHT);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("CROSSWALK"), ::ad::map::lane::ContactType::CROSSWALK);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::CROSSWALK"),
            ::ad::map::lane::ContactType::CROSSWALK);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("PRIO_TO_RIGHT"), ::ad::map::lane::ContactType::PRIO_TO_RIGHT);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::PRIO_TO_RIGHT"),
            ::ad::map::lane::ContactType::PRIO_TO_RIGHT);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("RIGHT_OF_WAY"), ::ad::map::lane::ContactType::RIGHT_OF_WAY);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::RIGHT_OF_WAY"),
            ::ad::map::lane::ContactType::RIGHT_OF_WAY);

  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("PRIO_TO_RIGHT_AND_STRAIGHT"),
            ::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT);
  ASSERT_EQ(fromString<::ad::map::lane::ContactType>("::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT"),
            ::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT);

  EXPECT_ANY_THROW({ fromString<::ad::map::lane::ContactType>("NOT A VALID ENUM LITERAL"); });
}

TEST(ContactTypeTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::lane::ContactType::INVALID), "::ad::map::lane::ContactType::INVALID");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::INVALID));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::INVALID));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::UNKNOWN), "::ad::map::lane::ContactType::UNKNOWN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::UNKNOWN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::UNKNOWN));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::FREE), "::ad::map::lane::ContactType::FREE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::FREE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::FREE));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::LANE_CHANGE), "::ad::map::lane::ContactType::LANE_CHANGE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::LANE_CHANGE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::LANE_CHANGE));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::LANE_CONTINUATION),
            "::ad::map::lane::ContactType::LANE_CONTINUATION");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::LANE_CONTINUATION));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::LANE_CONTINUATION));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::LANE_END), "::ad::map::lane::ContactType::LANE_END");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::LANE_END));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::LANE_END));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::SINGLE_POINT), "::ad::map::lane::ContactType::SINGLE_POINT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::SINGLE_POINT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::SINGLE_POINT));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::STOP), "::ad::map::lane::ContactType::STOP");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::STOP));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::STOP));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::STOP_ALL), "::ad::map::lane::ContactType::STOP_ALL");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::STOP_ALL));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::STOP_ALL));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::YIELD), "::ad::map::lane::ContactType::YIELD");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::YIELD));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::YIELD));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::GATE_BARRIER), "::ad::map::lane::ContactType::GATE_BARRIER");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::GATE_BARRIER));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::GATE_BARRIER));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::GATE_TOLBOOTH), "::ad::map::lane::ContactType::GATE_TOLBOOTH");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::GATE_TOLBOOTH));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::GATE_TOLBOOTH));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::GATE_SPIKES), "::ad::map::lane::ContactType::GATE_SPIKES");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::GATE_SPIKES));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::GATE_SPIKES));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::GATE_SPIKES_CONTRA),
            "::ad::map::lane::ContactType::GATE_SPIKES_CONTRA");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::GATE_SPIKES_CONTRA));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::GATE_SPIKES_CONTRA));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::CURB_UP), "::ad::map::lane::ContactType::CURB_UP");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::CURB_UP));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::CURB_UP));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::CURB_DOWN), "::ad::map::lane::ContactType::CURB_DOWN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::CURB_DOWN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::CURB_DOWN));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::SPEED_BUMP), "::ad::map::lane::ContactType::SPEED_BUMP");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::SPEED_BUMP));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::SPEED_BUMP));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::TRAFFIC_LIGHT), "::ad::map::lane::ContactType::TRAFFIC_LIGHT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::TRAFFIC_LIGHT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::TRAFFIC_LIGHT));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::CROSSWALK), "::ad::map::lane::ContactType::CROSSWALK");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::CROSSWALK));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::CROSSWALK));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::PRIO_TO_RIGHT), "::ad::map::lane::ContactType::PRIO_TO_RIGHT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::PRIO_TO_RIGHT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::PRIO_TO_RIGHT));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::RIGHT_OF_WAY), "::ad::map::lane::ContactType::RIGHT_OF_WAY");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::RIGHT_OF_WAY));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::RIGHT_OF_WAY));

  ASSERT_EQ(toString(::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT),
            "::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT));

  ASSERT_EQ(toString(static_cast<::ad::map::lane::ContactType>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::lane::ContactType>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(ContactTypeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::lane::ContactType value(::ad::map::lane::ContactType::INVALID);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
