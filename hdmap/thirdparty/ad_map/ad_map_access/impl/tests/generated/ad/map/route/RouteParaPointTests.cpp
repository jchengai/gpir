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
#include "ad/map/route/RouteParaPoint.hpp"

class RouteParaPointTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::route::RouteParaPoint value;
    ::ad::map::route::RoutePlanningCounter valueRoutePlanningCounter(
      std::numeric_limits<::ad::map::route::RoutePlanningCounter>::lowest());
    value.routePlanningCounter = valueRoutePlanningCounter;
    ::ad::map::route::SegmentCounter valueSegmentCountFromDestination(
      std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
    value.segmentCountFromDestination = valueSegmentCountFromDestination;
    ::ad::physics::ParametricValue valueParametricOffset(0.);
    value.parametricOffset = valueParametricOffset;
    mValue = value;
  }

  ::ad::map::route::RouteParaPoint mValue;
};

TEST_F(RouteParaPointTests, copyConstruction)
{
  ::ad::map::route::RouteParaPoint value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(RouteParaPointTests, moveConstruction)
{
  ::ad::map::route::RouteParaPoint tmpValue(mValue);
  ::ad::map::route::RouteParaPoint value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(RouteParaPointTests, copyAssignment)
{
  ::ad::map::route::RouteParaPoint value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(RouteParaPointTests, moveAssignment)
{
  ::ad::map::route::RouteParaPoint tmpValue(mValue);
  ::ad::map::route::RouteParaPoint value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(RouteParaPointTests, comparisonOperatorEqual)
{
  ::ad::map::route::RouteParaPoint valueA = mValue;
  ::ad::map::route::RouteParaPoint valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(RouteParaPointTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(RouteParaPointTests, comparisonOperatorRoutePlanningCounterDiffers)
{
  ::ad::map::route::RouteParaPoint valueA = mValue;
  ::ad::map::route::RoutePlanningCounter routePlanningCounter(
    std::numeric_limits<::ad::map::route::RoutePlanningCounter>::max());
  valueA.routePlanningCounter = routePlanningCounter;
  ::ad::map::route::RouteParaPoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(RouteParaPointTests, comparisonOperatorSegmentCountFromDestinationDiffers)
{
  ::ad::map::route::RouteParaPoint valueA = mValue;
  ::ad::map::route::SegmentCounter segmentCountFromDestination(
    std::numeric_limits<::ad::map::route::SegmentCounter>::max());
  valueA.segmentCountFromDestination = segmentCountFromDestination;
  ::ad::map::route::RouteParaPoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(RouteParaPointTests, comparisonOperatorParametricOffsetDiffers)
{
  ::ad::map::route::RouteParaPoint valueA = mValue;
  ::ad::physics::ParametricValue parametricOffset(1.);
  valueA.parametricOffset = parametricOffset;
  ::ad::map::route::RouteParaPoint valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
