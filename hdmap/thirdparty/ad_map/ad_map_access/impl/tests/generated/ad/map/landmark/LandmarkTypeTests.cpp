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
#include "ad/map/landmark/LandmarkType.hpp"

TEST(LandmarkTypeTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("INVALID"), ::ad::map::landmark::LandmarkType::INVALID);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::INVALID"),
            ::ad::map::landmark::LandmarkType::INVALID);

  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("UNKNOWN"), ::ad::map::landmark::LandmarkType::UNKNOWN);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::UNKNOWN"),
            ::ad::map::landmark::LandmarkType::UNKNOWN);

  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("TRAFFIC_SIGN"),
            ::ad::map::landmark::LandmarkType::TRAFFIC_SIGN);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::TRAFFIC_SIGN"),
            ::ad::map::landmark::LandmarkType::TRAFFIC_SIGN);

  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("TRAFFIC_LIGHT"),
            ::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT"),
            ::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT);

  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("POLE"), ::ad::map::landmark::LandmarkType::POLE);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::POLE"),
            ::ad::map::landmark::LandmarkType::POLE);

  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("GUIDE_POST"), ::ad::map::landmark::LandmarkType::GUIDE_POST);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::GUIDE_POST"),
            ::ad::map::landmark::LandmarkType::GUIDE_POST);

  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("TREE"), ::ad::map::landmark::LandmarkType::TREE);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::TREE"),
            ::ad::map::landmark::LandmarkType::TREE);

  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("STREET_LAMP"),
            ::ad::map::landmark::LandmarkType::STREET_LAMP);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::STREET_LAMP"),
            ::ad::map::landmark::LandmarkType::STREET_LAMP);

  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("POSTBOX"), ::ad::map::landmark::LandmarkType::POSTBOX);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::POSTBOX"),
            ::ad::map::landmark::LandmarkType::POSTBOX);

  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("MANHOLE"), ::ad::map::landmark::LandmarkType::MANHOLE);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::MANHOLE"),
            ::ad::map::landmark::LandmarkType::MANHOLE);

  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("POWERCABINET"),
            ::ad::map::landmark::LandmarkType::POWERCABINET);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::POWERCABINET"),
            ::ad::map::landmark::LandmarkType::POWERCABINET);

  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("FIRE_HYDRANT"),
            ::ad::map::landmark::LandmarkType::FIRE_HYDRANT);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::FIRE_HYDRANT"),
            ::ad::map::landmark::LandmarkType::FIRE_HYDRANT);

  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("BOLLARD"), ::ad::map::landmark::LandmarkType::BOLLARD);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::BOLLARD"),
            ::ad::map::landmark::LandmarkType::BOLLARD);

  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("OTHER"), ::ad::map::landmark::LandmarkType::OTHER);
  ASSERT_EQ(fromString<::ad::map::landmark::LandmarkType>("::ad::map::landmark::LandmarkType::OTHER"),
            ::ad::map::landmark::LandmarkType::OTHER);

  EXPECT_ANY_THROW({ fromString<::ad::map::landmark::LandmarkType>("NOT A VALID ENUM LITERAL"); });
}

TEST(LandmarkTypeTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::INVALID), "::ad::map::landmark::LandmarkType::INVALID");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::INVALID));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::INVALID));

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::UNKNOWN), "::ad::map::landmark::LandmarkType::UNKNOWN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::UNKNOWN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::UNKNOWN));

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::TRAFFIC_SIGN),
            "::ad::map::landmark::LandmarkType::TRAFFIC_SIGN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::TRAFFIC_SIGN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::TRAFFIC_SIGN));

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT),
            "::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT));

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::POLE), "::ad::map::landmark::LandmarkType::POLE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::POLE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::POLE));

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::GUIDE_POST), "::ad::map::landmark::LandmarkType::GUIDE_POST");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::GUIDE_POST));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::GUIDE_POST));

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::TREE), "::ad::map::landmark::LandmarkType::TREE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::TREE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::TREE));

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::STREET_LAMP), "::ad::map::landmark::LandmarkType::STREET_LAMP");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::STREET_LAMP));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::STREET_LAMP));

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::POSTBOX), "::ad::map::landmark::LandmarkType::POSTBOX");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::POSTBOX));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::POSTBOX));

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::MANHOLE), "::ad::map::landmark::LandmarkType::MANHOLE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::MANHOLE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::MANHOLE));

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::POWERCABINET),
            "::ad::map::landmark::LandmarkType::POWERCABINET");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::POWERCABINET));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::POWERCABINET));

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::FIRE_HYDRANT),
            "::ad::map::landmark::LandmarkType::FIRE_HYDRANT");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::FIRE_HYDRANT));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::FIRE_HYDRANT));

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::BOLLARD), "::ad::map::landmark::LandmarkType::BOLLARD");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::BOLLARD));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::BOLLARD));

  ASSERT_EQ(toString(::ad::map::landmark::LandmarkType::OTHER), "::ad::map::landmark::LandmarkType::OTHER");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::OTHER));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::landmark::LandmarkType::OTHER));

  ASSERT_EQ(toString(static_cast<::ad::map::landmark::LandmarkType>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::landmark::LandmarkType>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(LandmarkTypeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::landmark::LandmarkType value(::ad::map::landmark::LandmarkType::INVALID);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
