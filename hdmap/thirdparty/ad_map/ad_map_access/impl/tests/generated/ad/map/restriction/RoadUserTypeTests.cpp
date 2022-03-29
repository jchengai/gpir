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
#include "ad/map/restriction/RoadUserType.hpp"

TEST(RoadUserTypeTests, testFromString)
{
  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("INVALID"), ::ad::map::restriction::RoadUserType::INVALID);
  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("::ad::map::restriction::RoadUserType::INVALID"),
            ::ad::map::restriction::RoadUserType::INVALID);

  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("UNKNOWN"), ::ad::map::restriction::RoadUserType::UNKNOWN);
  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("::ad::map::restriction::RoadUserType::UNKNOWN"),
            ::ad::map::restriction::RoadUserType::UNKNOWN);

  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("CAR"), ::ad::map::restriction::RoadUserType::CAR);
  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("::ad::map::restriction::RoadUserType::CAR"),
            ::ad::map::restriction::RoadUserType::CAR);

  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("BUS"), ::ad::map::restriction::RoadUserType::BUS);
  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("::ad::map::restriction::RoadUserType::BUS"),
            ::ad::map::restriction::RoadUserType::BUS);

  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("TRUCK"), ::ad::map::restriction::RoadUserType::TRUCK);
  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("::ad::map::restriction::RoadUserType::TRUCK"),
            ::ad::map::restriction::RoadUserType::TRUCK);

  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("PEDESTRIAN"),
            ::ad::map::restriction::RoadUserType::PEDESTRIAN);
  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("::ad::map::restriction::RoadUserType::PEDESTRIAN"),
            ::ad::map::restriction::RoadUserType::PEDESTRIAN);

  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("MOTORBIKE"),
            ::ad::map::restriction::RoadUserType::MOTORBIKE);
  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("::ad::map::restriction::RoadUserType::MOTORBIKE"),
            ::ad::map::restriction::RoadUserType::MOTORBIKE);

  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("BICYCLE"), ::ad::map::restriction::RoadUserType::BICYCLE);
  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("::ad::map::restriction::RoadUserType::BICYCLE"),
            ::ad::map::restriction::RoadUserType::BICYCLE);

  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("CAR_ELECTRIC"),
            ::ad::map::restriction::RoadUserType::CAR_ELECTRIC);
  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("::ad::map::restriction::RoadUserType::CAR_ELECTRIC"),
            ::ad::map::restriction::RoadUserType::CAR_ELECTRIC);

  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("CAR_HYBRID"),
            ::ad::map::restriction::RoadUserType::CAR_HYBRID);
  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("::ad::map::restriction::RoadUserType::CAR_HYBRID"),
            ::ad::map::restriction::RoadUserType::CAR_HYBRID);

  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("CAR_PETROL"),
            ::ad::map::restriction::RoadUserType::CAR_PETROL);
  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("::ad::map::restriction::RoadUserType::CAR_PETROL"),
            ::ad::map::restriction::RoadUserType::CAR_PETROL);

  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("CAR_DIESEL"),
            ::ad::map::restriction::RoadUserType::CAR_DIESEL);
  ASSERT_EQ(fromString<::ad::map::restriction::RoadUserType>("::ad::map::restriction::RoadUserType::CAR_DIESEL"),
            ::ad::map::restriction::RoadUserType::CAR_DIESEL);

  EXPECT_ANY_THROW({ fromString<::ad::map::restriction::RoadUserType>("NOT A VALID ENUM LITERAL"); });
}

TEST(RoadUserTypeTests, testToString)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();
  int32_t maxValue = std::numeric_limits<int32_t>::min();

  ASSERT_EQ(toString(::ad::map::restriction::RoadUserType::INVALID), "::ad::map::restriction::RoadUserType::INVALID");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::INVALID));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::INVALID));

  ASSERT_EQ(toString(::ad::map::restriction::RoadUserType::UNKNOWN), "::ad::map::restriction::RoadUserType::UNKNOWN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::UNKNOWN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::UNKNOWN));

  ASSERT_EQ(toString(::ad::map::restriction::RoadUserType::CAR), "::ad::map::restriction::RoadUserType::CAR");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR));

  ASSERT_EQ(toString(::ad::map::restriction::RoadUserType::BUS), "::ad::map::restriction::RoadUserType::BUS");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::BUS));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::BUS));

  ASSERT_EQ(toString(::ad::map::restriction::RoadUserType::TRUCK), "::ad::map::restriction::RoadUserType::TRUCK");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::TRUCK));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::TRUCK));

  ASSERT_EQ(toString(::ad::map::restriction::RoadUserType::PEDESTRIAN),
            "::ad::map::restriction::RoadUserType::PEDESTRIAN");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::PEDESTRIAN));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::PEDESTRIAN));

  ASSERT_EQ(toString(::ad::map::restriction::RoadUserType::MOTORBIKE),
            "::ad::map::restriction::RoadUserType::MOTORBIKE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::MOTORBIKE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::MOTORBIKE));

  ASSERT_EQ(toString(::ad::map::restriction::RoadUserType::BICYCLE), "::ad::map::restriction::RoadUserType::BICYCLE");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::BICYCLE));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::BICYCLE));

  ASSERT_EQ(toString(::ad::map::restriction::RoadUserType::CAR_ELECTRIC),
            "::ad::map::restriction::RoadUserType::CAR_ELECTRIC");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR_ELECTRIC));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR_ELECTRIC));

  ASSERT_EQ(toString(::ad::map::restriction::RoadUserType::CAR_HYBRID),
            "::ad::map::restriction::RoadUserType::CAR_HYBRID");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR_HYBRID));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR_HYBRID));

  ASSERT_EQ(toString(::ad::map::restriction::RoadUserType::CAR_PETROL),
            "::ad::map::restriction::RoadUserType::CAR_PETROL");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR_PETROL));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR_PETROL));

  ASSERT_EQ(toString(::ad::map::restriction::RoadUserType::CAR_DIESEL),
            "::ad::map::restriction::RoadUserType::CAR_DIESEL");
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR_DIESEL));
  maxValue = std::max(maxValue, static_cast<int32_t>(::ad::map::restriction::RoadUserType::CAR_DIESEL));

  ASSERT_EQ(toString(static_cast<::ad::map::restriction::RoadUserType>(minValue - 1)), "UNKNOWN ENUM VALUE");
  ASSERT_EQ(toString(static_cast<::ad::map::restriction::RoadUserType>(maxValue + 1)), "UNKNOWN ENUM VALUE");
}

TEST(RoadUserTypeTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::restriction::RoadUserType value(::ad::map::restriction::RoadUserType::INVALID);
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
