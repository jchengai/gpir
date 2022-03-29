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
#include "ad/map/match/ObjectReferencePoints.hpp"

TEST(ObjectReferencePointsTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::match::ObjectReferencePoints>("FrontLeft"),
            ::ad::map::match::ObjectReferencePoints::FrontLeft);
  ASSERT_EQ(fromString<::ad::map::match::ObjectReferencePoints>("::ad::map::match::ObjectReferencePoints::FrontLeft"),
            ::ad::map::match::ObjectReferencePoints::FrontLeft);

  ASSERT_EQ(fromString<::ad::map::match::ObjectReferencePoints>("FrontRight"),
            ::ad::map::match::ObjectReferencePoints::FrontRight);
  ASSERT_EQ(fromString<::ad::map::match::ObjectReferencePoints>("::ad::map::match::ObjectReferencePoints::FrontRight"),
            ::ad::map::match::ObjectReferencePoints::FrontRight);

  ASSERT_EQ(fromString<::ad::map::match::ObjectReferencePoints>("RearLeft"),
            ::ad::map::match::ObjectReferencePoints::RearLeft);
  ASSERT_EQ(fromString<::ad::map::match::ObjectReferencePoints>("::ad::map::match::ObjectReferencePoints::RearLeft"),
            ::ad::map::match::ObjectReferencePoints::RearLeft);

  ASSERT_EQ(fromString<::ad::map::match::ObjectReferencePoints>("RearRight"),
            ::ad::map::match::ObjectReferencePoints::RearRight);
  ASSERT_EQ(fromString<::ad::map::match::ObjectReferencePoints>("::ad::map::match::ObjectReferencePoints::RearRight"),
            ::ad::map::match::ObjectReferencePoints::RearRight);

  ASSERT_EQ(fromString<::ad::map::match::ObjectReferencePoints>("Center"),
            ::ad::map::match::ObjectReferencePoints::Center);
  ASSERT_EQ(fromString<::ad::map::match::ObjectReferencePoints>("::ad::map::match::ObjectReferencePoints::Center"),
            ::ad::map::match::ObjectReferencePoints::Center);

  ASSERT_EQ(fromString<::ad::map::match::ObjectReferencePoints>("NumPoints"),
            ::ad::map::match::ObjectReferencePoints::NumPoints);
  ASSERT_EQ(fromString<::ad::map::match::ObjectReferencePoints>("::ad::map::match::ObjectReferencePoints::NumPoints"),
            ::ad::map::match::ObjectReferencePoints::NumPoints);

  EXPECT_ANY_THROW({ fromString<::ad::map::match::ObjectReferencePoints>("NOT A VALID ENUM LITERAL"); });
}

TEST(ObjectReferencePointsTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::match::ObjectReferencePoints::FrontLeft),
            "::ad::map::match::ObjectReferencePoints::FrontLeft");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::FrontLeft));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::FrontLeft));

  ASSERT_EQ(toString(::ad::map::match::ObjectReferencePoints::FrontRight),
            "::ad::map::match::ObjectReferencePoints::FrontRight");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::FrontRight));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::FrontRight));

  ASSERT_EQ(toString(::ad::map::match::ObjectReferencePoints::RearLeft),
            "::ad::map::match::ObjectReferencePoints::RearLeft");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::RearLeft));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::RearLeft));

  ASSERT_EQ(toString(::ad::map::match::ObjectReferencePoints::RearRight),
            "::ad::map::match::ObjectReferencePoints::RearRight");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::RearRight));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::RearRight));

  ASSERT_EQ(toString(::ad::map::match::ObjectReferencePoints::Center),
            "::ad::map::match::ObjectReferencePoints::Center");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::Center));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::Center));

  ASSERT_EQ(toString(::ad::map::match::ObjectReferencePoints::NumPoints),
            "::ad::map::match::ObjectReferencePoints::NumPoints");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::NumPoints));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::NumPoints));

  ASSERT_EQ(toString(static_cast<::ad::map::match::ObjectReferencePoints>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::match::ObjectReferencePoints>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(ObjectReferencePointsTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::match::ObjectReferencePoints value(::ad::map::match::ObjectReferencePoints::FrontLeft);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
