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
#include "ad/map/lane/LaneId.hpp"

TEST(LaneIdTests, minIsValid)
{
  EXPECT_TRUE(::ad::map::lane::LaneId::getMin().isValid());
}

TEST(LaneIdTests, aboveMinIsValid)
{
  ::ad::map::lane::LaneId value(::ad::map::lane::LaneId::cMinValue + 1);
  EXPECT_TRUE(value.isValid());
}

TEST(LaneIdTests, maxIsValid)
{
  EXPECT_TRUE(::ad::map::lane::LaneId::getMax().isValid());
}

TEST(LaneIdTests, belowMaxIsValid)
{
  ::ad::map::lane::LaneId value(::ad::map::lane::LaneId::cMaxValue - 1);
  EXPECT_TRUE(value.isValid());
}

#if (AD_MAP_LANE_LANEID_THROWS_EXCEPTION == 1)
TEST(LaneIdTests, ensureValidNonZeroThrowsOnInvalid)
{
  ::ad::map::lane::LaneId value;
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}

TEST(LaneIdTests, ensureValidNonZeroThrowsOnZero)
{
  ::ad::map::lane::LaneId value(0.);
  EXPECT_THROW(value.ensureValidNonZero(), std::out_of_range);
}
#endif

TEST(LaneIdTestsStd, numericLimitsLowestIsMin)
{
  EXPECT_EQ(static_cast<uint64_t>(::ad::map::lane::LaneId::getMin()),
            static_cast<uint64_t>(std::numeric_limits<::ad::map::lane::LaneId>::lowest()));
}

TEST(LaneIdTestsStd, numericLimitsMaxIsMax)
{
  EXPECT_EQ(static_cast<uint64_t>(::ad::map::lane::LaneId::getMax()),
            static_cast<uint64_t>(std::numeric_limits<::ad::map::lane::LaneId>::max()));
}

TEST(LaneIdTests, copyConstructionFromValidValue)
{
  ::ad::map::lane::LaneId const validValue(::ad::map::lane::LaneId::cMinValue);
  ::ad::map::lane::LaneId value(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_EQ(static_cast<uint64_t>(validValue), static_cast<uint64_t>(value));
}

TEST(LaneIdTests, moveConstructionFromValidValue)
{
  ::ad::map::lane::LaneId validValue(::ad::map::lane::LaneId::cMinValue);
  ::ad::map::lane::LaneId value(std::move(validValue));
  EXPECT_TRUE(value.isValid());
  EXPECT_EQ(::ad::map::lane::LaneId::cMinValue, static_cast<uint64_t>(value));
}

TEST(LaneIdTests, assignmentFromValidValue)
{
  ::ad::map::lane::LaneId const validValue(::ad::map::lane::LaneId::cMinValue);
  ::ad::map::lane::LaneId value;
  value = validValue;
  EXPECT_TRUE(value.isValid());
  EXPECT_EQ(static_cast<uint64_t>(validValue), static_cast<uint64_t>(value));
}

TEST(LaneIdTests, moveAssignmentFromValidValue)
{
  ::ad::map::lane::LaneId validValue(::ad::map::lane::LaneId::cMinValue);
  ::ad::map::lane::LaneId value;
  value = std::move(validValue);
  EXPECT_TRUE(value.isValid());
  EXPECT_EQ(::ad::map::lane::LaneId::cMinValue, static_cast<uint64_t>(value));
}

TEST(LaneIdTests, selfAssignment)
{
  ::ad::map::lane::LaneId value(::ad::map::lane::LaneId::cMinValue);
  EXPECT_TRUE(value.isValid());
  value = value;
  EXPECT_TRUE(value.isValid());
}

TEST(LaneIdTests, ostreamOperatorTest)
{
  std::stringstream stream;
  ::ad::map::lane::LaneId value;
  stream << value;
  ASSERT_GT(stream.str().size(), 0u);
}

TEST(LaneIdTests, comparisonOperators)
{
  ::ad::map::lane::LaneId minValue(::ad::map::lane::LaneId::cMinValue);
  ::ad::map::lane::LaneId maxValue(::ad::map::lane::LaneId::cMaxValue);

  ASSERT_TRUE(minValue == minValue);
  ASSERT_TRUE(minValue != maxValue);
  ASSERT_TRUE(minValue < maxValue);
  ASSERT_TRUE(minValue <= minValue);
  ASSERT_TRUE(maxValue > minValue);
  ASSERT_TRUE(maxValue >= minValue);
}

TEST(LaneIdTests, arithmeticOperators)
{
  ::ad::map::lane::LaneId minValue(::ad::map::lane::LaneId::cMinValue);
  ::ad::map::lane::LaneId maxValue(::ad::map::lane::LaneId::cMaxValue);
  ::ad::map::lane::LaneId result;

  ASSERT_EQ(static_cast<uint64_t>(minValue + minValue),
            ::ad::map::lane::LaneId::cMinValue + ::ad::map::lane::LaneId::cMinValue);
  result = minValue;
  result += minValue;
  ASSERT_EQ(static_cast<uint64_t>(result), ::ad::map::lane::LaneId::cMinValue + ::ad::map::lane::LaneId::cMinValue);
  ASSERT_EQ(static_cast<uint64_t>(maxValue - minValue),
            ::ad::map::lane::LaneId::cMaxValue - ::ad::map::lane::LaneId::cMinValue);
  result = maxValue;
  result -= minValue;
  ASSERT_EQ(static_cast<uint64_t>(result), ::ad::map::lane::LaneId::cMaxValue - ::ad::map::lane::LaneId::cMinValue);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
