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
#include "ad/map/match/LaneOccupiedRegion.hpp"

class LaneOccupiedRegionTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::match::LaneOccupiedRegion value;
    ::ad::map::lane::LaneId valueLaneId(1);
    value.laneId = valueLaneId;
    ::ad::physics::ParametricRange valueLongitudinalRange;
    ::ad::physics::ParametricValue valueLongitudinalRangeMinimum(0.);
    valueLongitudinalRange.minimum = valueLongitudinalRangeMinimum;
    ::ad::physics::ParametricValue valueLongitudinalRangeMaximum(0.);
    valueLongitudinalRange.maximum = valueLongitudinalRangeMaximum;
    valueLongitudinalRange.maximum = valueLongitudinalRange.minimum;
    valueLongitudinalRange.minimum = valueLongitudinalRange.maximum;
    value.longitudinalRange = valueLongitudinalRange;
    ::ad::physics::ParametricRange valueLateralRange;
    ::ad::physics::ParametricValue valueLateralRangeMinimum(0.);
    valueLateralRange.minimum = valueLateralRangeMinimum;
    ::ad::physics::ParametricValue valueLateralRangeMaximum(0.);
    valueLateralRange.maximum = valueLateralRangeMaximum;
    valueLateralRange.maximum = valueLateralRange.minimum;
    valueLateralRange.minimum = valueLateralRange.maximum;
    value.lateralRange = valueLateralRange;
    mValue = value;
  }

  ::ad::map::match::LaneOccupiedRegion mValue;
};

TEST_F(LaneOccupiedRegionTests, copyConstruction)
{
  ::ad::map::match::LaneOccupiedRegion value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneOccupiedRegionTests, moveConstruction)
{
  ::ad::map::match::LaneOccupiedRegion tmpValue(mValue);
  ::ad::map::match::LaneOccupiedRegion value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneOccupiedRegionTests, copyAssignment)
{
  ::ad::map::match::LaneOccupiedRegion value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneOccupiedRegionTests, moveAssignment)
{
  ::ad::map::match::LaneOccupiedRegion tmpValue(mValue);
  ::ad::map::match::LaneOccupiedRegion value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneOccupiedRegionTests, comparisonOperatorEqual)
{
  ::ad::map::match::LaneOccupiedRegion valueA = mValue;
  ::ad::map::match::LaneOccupiedRegion valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(LaneOccupiedRegionTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(LaneOccupiedRegionTests, comparisonOperatorLaneIdDiffers)
{
  ::ad::map::match::LaneOccupiedRegion valueA = mValue;
  ::ad::map::lane::LaneId laneId(std::numeric_limits<::ad::map::lane::LaneId>::max());
  valueA.laneId = laneId;
  ::ad::map::match::LaneOccupiedRegion valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneOccupiedRegionTests, comparisonOperatorLongitudinalRangeDiffers)
{
  ::ad::map::match::LaneOccupiedRegion valueA = mValue;
  ::ad::physics::ParametricRange longitudinalRange;
  ::ad::physics::ParametricValue longitudinalRangeMinimum(1.);
  longitudinalRange.minimum = longitudinalRangeMinimum;
  ::ad::physics::ParametricValue longitudinalRangeMaximum(1.);
  longitudinalRange.maximum = longitudinalRangeMaximum;
  longitudinalRange.maximum = longitudinalRange.minimum;
  longitudinalRange.minimum = longitudinalRange.maximum;
  valueA.longitudinalRange = longitudinalRange;
  ::ad::map::match::LaneOccupiedRegion valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneOccupiedRegionTests, comparisonOperatorLateralRangeDiffers)
{
  ::ad::map::match::LaneOccupiedRegion valueA = mValue;
  ::ad::physics::ParametricRange lateralRange;
  ::ad::physics::ParametricValue lateralRangeMinimum(1.);
  lateralRange.minimum = lateralRangeMinimum;
  ::ad::physics::ParametricValue lateralRangeMaximum(1.);
  lateralRange.maximum = lateralRangeMaximum;
  lateralRange.maximum = lateralRange.minimum;
  lateralRange.minimum = lateralRange.maximum;
  valueA.lateralRange = lateralRange;
  ::ad::map::match::LaneOccupiedRegion valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
