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
#include "ad/map/route/LaneInterval.hpp"

class LaneIntervalTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::route::LaneInterval value;
    ::ad::map::lane::LaneId valueLaneId(1);
    value.laneId = valueLaneId;
    ::ad::physics::ParametricValue valueStart(0.);
    value.start = valueStart;
    ::ad::physics::ParametricValue valueEnd(0.);
    value.end = valueEnd;
    bool valueWrongWay{true};
    value.wrongWay = valueWrongWay;
    mValue = value;
  }

  ::ad::map::route::LaneInterval mValue;
};

TEST_F(LaneIntervalTests, copyConstruction)
{
  ::ad::map::route::LaneInterval value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneIntervalTests, moveConstruction)
{
  ::ad::map::route::LaneInterval tmpValue(mValue);
  ::ad::map::route::LaneInterval value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneIntervalTests, copyAssignment)
{
  ::ad::map::route::LaneInterval value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneIntervalTests, moveAssignment)
{
  ::ad::map::route::LaneInterval tmpValue(mValue);
  ::ad::map::route::LaneInterval value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneIntervalTests, comparisonOperatorEqual)
{
  ::ad::map::route::LaneInterval valueA = mValue;
  ::ad::map::route::LaneInterval valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(LaneIntervalTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(LaneIntervalTests, comparisonOperatorLaneIdDiffers)
{
  ::ad::map::route::LaneInterval valueA = mValue;
  ::ad::map::lane::LaneId laneId(std::numeric_limits<::ad::map::lane::LaneId>::max());
  valueA.laneId = laneId;
  ::ad::map::route::LaneInterval valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneIntervalTests, comparisonOperatorStartDiffers)
{
  ::ad::map::route::LaneInterval valueA = mValue;
  ::ad::physics::ParametricValue start(1.);
  valueA.start = start;
  ::ad::map::route::LaneInterval valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneIntervalTests, comparisonOperatorEndDiffers)
{
  ::ad::map::route::LaneInterval valueA = mValue;
  ::ad::physics::ParametricValue end(1.);
  valueA.end = end;
  ::ad::map::route::LaneInterval valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneIntervalTests, comparisonOperatorWrongWayDiffers)
{
  ::ad::map::route::LaneInterval valueA = mValue;
  bool wrongWay{false};
  valueA.wrongWay = wrongWay;
  ::ad::map::route::LaneInterval valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
