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

#include <gtest/gtest.h>

#include <limits>

#include "ad/map/match/ObjectReferencePointsValidInputRange.hpp"

TEST(ObjectReferencePointsValidInputRangeTests, testValidInputRangeValid)
{
  ASSERT_TRUE(withinValidInputRange(::ad::map::match::ObjectReferencePoints::FrontLeft));
  ASSERT_TRUE(withinValidInputRange(::ad::map::match::ObjectReferencePoints::FrontRight));
  ASSERT_TRUE(withinValidInputRange(::ad::map::match::ObjectReferencePoints::RearLeft));
  ASSERT_TRUE(withinValidInputRange(::ad::map::match::ObjectReferencePoints::RearRight));
  ASSERT_TRUE(withinValidInputRange(::ad::map::match::ObjectReferencePoints::Center));
  ASSERT_TRUE(withinValidInputRange(::ad::map::match::ObjectReferencePoints::NumPoints));
}

TEST(ObjectReferencePointsValidInputRangeTests, testValidInputRangeInvalid)
{
  int32_t minValue = std::numeric_limits<int32_t>::max();

  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::FrontLeft));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::FrontRight));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::RearLeft));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::RearRight));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::Center));
  minValue = std::min(minValue, static_cast<int32_t>(::ad::map::match::ObjectReferencePoints::NumPoints));

  ASSERT_FALSE(withinValidInputRange(static_cast<::ad::map::match::ObjectReferencePoints>(minValue - 1)));
}
