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
#include "ad/map/match/LanePoint.hpp"

class LanePointTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::match::LanePoint value;
    ::ad::map::point::ParaPoint valueParaPoint;
    ::ad::map::lane::LaneId valueParaPointLaneId(1);
    valueParaPoint.laneId = valueParaPointLaneId;
    ::ad::physics::ParametricValue valueParaPointParametricOffset(0.);
    valueParaPoint.parametricOffset = valueParaPointParametricOffset;
    value.paraPoint = valueParaPoint;
    ::ad::physics::RatioValue valueLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
    value.lateralT = valueLateralT;
    ::ad::physics::Distance valueLaneLength(-1e9);
    value.laneLength = valueLaneLength;
    ::ad::physics::Distance valueLaneWidth(-1e9);
    value.laneWidth = valueLaneWidth;
    mValue = value;
  }

  ::ad::map::match::LanePoint mValue;
};

TEST_F(LanePointTests, copyConstruction)
{
  ::ad::map::match::LanePoint value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(LanePointTests, moveConstruction)
{
  ::ad::map::match::LanePoint tmpValue(mValue);
  ::ad::map::match::LanePoint value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(LanePointTests, copyAssignment)
{
  ::ad::map::match::LanePoint value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(LanePointTests, moveAssignment)
{
  ::ad::map::match::LanePoint tmpValue(mValue);
  ::ad::map::match::LanePoint value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(LanePointTests, comparisonOperatorEqual)
{
  ::ad::map::match::LanePoint valueA = mValue;
  ::ad::map::match::LanePoint valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(LanePointTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(LanePointTests, comparisonOperatorParaPointDiffers)
{
  ::ad::map::match::LanePoint valueA = mValue;
  ::ad::map::point::ParaPoint paraPoint;
  ::ad::map::lane::LaneId paraPointLaneId(std::numeric_limits<::ad::map::lane::LaneId>::max());
  paraPoint.laneId = paraPointLaneId;
  ::ad::physics::ParametricValue paraPointParametricOffset(1.);
  paraPoint.parametricOffset = paraPointParametricOffset;
  valueA.paraPoint = paraPoint;
  ::ad::map::match::LanePoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LanePointTests, comparisonOperatorLateralTDiffers)
{
  ::ad::map::match::LanePoint valueA = mValue;
  ::ad::physics::RatioValue lateralT(std::numeric_limits<::ad::physics::RatioValue>::max());
  valueA.lateralT = lateralT;
  ::ad::map::match::LanePoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LanePointTests, comparisonOperatorLaneLengthDiffers)
{
  ::ad::map::match::LanePoint valueA = mValue;
  ::ad::physics::Distance laneLength(1e9);
  valueA.laneLength = laneLength;
  ::ad::map::match::LanePoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LanePointTests, comparisonOperatorLaneWidthDiffers)
{
  ::ad::map::match::LanePoint valueA = mValue;
  ::ad::physics::Distance laneWidth(1e9);
  valueA.laneWidth = laneWidth;
  ::ad::map::match::LanePoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
