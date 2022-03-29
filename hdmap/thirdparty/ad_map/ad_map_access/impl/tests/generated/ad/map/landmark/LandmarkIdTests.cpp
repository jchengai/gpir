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
#include "ad/map/landmark/LandmarkId.hpp"

TEST(LandmarkIdTests, minIsValid)
{
  EXPECT_TRUE(::ad::map::landmark::LandmarkId::getMin().isValid());
}

TEST(LandmarkIdTests, aboveMinIsValid)
{
  ::ad::map::landmark::LandmarkId value(::ad::map::landmark::LandmarkId::cMinValue + 1);
  EXPECT_TRUE(value.isValid());
}

TEST(LandmarkIdTests, maxIsValid)
{
  EXPECT_TRUE(::ad::map::landmark::LandmarkId::getMax().isValid());
}

TEST(LandmarkIdTests, belowMaxIsValid)
{
  ::ad::map::landmark::LandmarkId value(::ad::map::landmark::LandmarkId::cMaxValue - 1);
  EXPECT_TRUE(value.isValid());
}

#if (AD_MAP_LANDMARK_LANDMARKID_THROWS_EXCEPTION == 1)
TEST(LandmarkIdTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::map::landmark::LandmarkId value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(LandmarkIdTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::map::landmark::LandmarkId value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(LandmarkIdTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_EQ(static_cast<uint64_t>(::ad::map::landmark::LandmarkId::getMin()),
            static_cast<uint64_t>(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest()));
}

TEST(LandmarkIdTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_EQ(static_cast<uint64_t>(::ad::map::landmark::LandmarkId::getMax()),
            static_cast<uint64_t>(std::numeric_limits<::ad::map::landmark::LandmarkId>::max()));
}

TEST(LandmarkIdTests, copyConstructionFromValidValue)
{
  ::ad::map::landmark::LandmarkId const validValue(::ad::map::landmark::LandmarkId::cMinValue);
  ::ad::map::landmark::LandmarkId value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_EQ(static_cast<uint64_t>(validValue), static_cast<uint64_t>(value));
}

TEST(LandmarkIdTests, moveConstructionFromValidValue)
{
  ::ad::map::landmark::LandmarkId validValue(::ad::map::landmark::LandmarkId::cMinValue);
  ::ad::map::landmark::LandmarkId value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_EQ(::ad::map::landmark::LandmarkId::cMinValue, static_cast<uint64_t>(value));
}

TEST(LandmarkIdTests, assignmentFromValidValue)
{
  ::ad::map::landmark::LandmarkId const validValue(::ad::map::landmark::LandmarkId::cMinValue);
  ::ad::map::landmark::LandmarkId value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_EQ(static_cast<uint64_t>(validValue), static_cast<uint64_t>(value));
}

TEST(LandmarkIdTests, moveAssignmentFromValidValue)
{
  ::ad::map::landmark::LandmarkId validValue(::ad::map::landmark::LandmarkId::cMinValue);
  ::ad::map::landmark::LandmarkId value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_EQ(::ad::map::landmark::LandmarkId::cMinValue, static_cast<uint64_t>(value));
}

TEST(LandmarkIdTests, selfAssignment)
{
  ::ad::map::landmark::LandmarkId value(::ad::map::landmark::LandmarkId::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(LandmarkIdTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::landmark::LandmarkId value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

TEST(LandmarkIdTests, comparisonOperators)
{
  ::ad::map::landmark::LandmarkId minValue(::ad::map::landmark::LandmarkId::cMinValue);
  ::ad::map::landmark::LandmarkId maxValue(::ad::map::landmark::LandmarkId::cMaxValue);

  ASSERT_TRUE(minValue == minValue);
  ASSERT_TRUE(minValue != maxValue);
  ASSERT_TRUE(minValue < maxValue);
  ASSERT_TRUE(minValue <= minValue);
  ASSERT_TRUE(maxValue > minValue);
  ASSERT_TRUE(maxValue >= minValue);
}

TEST(LandmarkIdTests, arithmeticOperators)
{
  ::ad::map::landmark::LandmarkId minValue(::ad::map::landmark::LandmarkId::cMinValue);
  ::ad::map::landmark::LandmarkId maxValue(::ad::map::landmark::LandmarkId::cMaxValue);
  ::ad::map::landmark::LandmarkId result;

  ASSERT_EQ(static_cast<uint64_t>(minValue + minValue),
            ::ad::map::landmark::LandmarkId::cMinValue + ::ad::map::landmark::LandmarkId::cMinValue);
  result = minValue;
  result += minValue;
  ASSERT_EQ(static_cast<uint64_t>(result),
            ::ad::map::landmark::LandmarkId::cMinValue + ::ad::map::landmark::LandmarkId::cMinValue);
  ASSERT_EQ(static_cast<uint64_t>(maxValue - minValue),
            ::ad::map::landmark::LandmarkId::cMaxValue - ::ad::map::landmark::LandmarkId::cMinValue);
  result = maxValue;
  result -= minValue;
  ASSERT_EQ(static_cast<uint64_t>(result),
            ::ad::map::landmark::LandmarkId::cMaxValue - ::ad::map::landmark::LandmarkId::cMinValue);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
