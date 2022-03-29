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
#include "ad/map/match/MapMatchedPosition.hpp"

class MapMatchedPositionTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::match::MapMatchedPosition value;
    ::ad::map::match::LanePoint valueLanePoint;
    ::ad::map::point::ParaPoint valueLanePointParaPoint;
    ::ad::map::lane::LaneId valueLanePointParaPointLaneId(1);
    valueLanePointParaPoint.laneId = valueLanePointParaPointLaneId;
    ::ad::physics::ParametricValue valueLanePointParaPointParametricOffset(0.);
    valueLanePointParaPoint.parametricOffset = valueLanePointParaPointParametricOffset;
    valueLanePoint.paraPoint = valueLanePointParaPoint;
    ::ad::physics::RatioValue valueLanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::lowest());
    valueLanePoint.lateralT = valueLanePointLateralT;
    ::ad::physics::Distance valueLanePointLaneLength(-1e9);
    valueLanePoint.laneLength = valueLanePointLaneLength;
    ::ad::physics::Distance valueLanePointLaneWidth(-1e9);
    valueLanePoint.laneWidth = valueLanePointLaneWidth;
    value.lanePoint = valueLanePoint;
    ::ad::map::match::MapMatchedPositionType valueType(::ad::map::match::MapMatchedPositionType::INVALID);
    value.type = valueType;
    ::ad::map::point::ECEFPoint valueMatchedPoint;
    ::ad::map::point::ECEFCoordinate valueMatchedPointX(-6400000);
    valueMatchedPoint.x = valueMatchedPointX;
    ::ad::map::point::ECEFCoordinate valueMatchedPointY(-6400000);
    valueMatchedPoint.y = valueMatchedPointY;
    ::ad::map::point::ECEFCoordinate valueMatchedPointZ(-6400000);
    valueMatchedPoint.z = valueMatchedPointZ;
    value.matchedPoint = valueMatchedPoint;
    ::ad::physics::Probability valueProbability(0.);
    value.probability = valueProbability;
    ::ad::map::point::ECEFPoint valueQueryPoint;
    ::ad::map::point::ECEFCoordinate valueQueryPointX(-6400000);
    valueQueryPoint.x = valueQueryPointX;
    ::ad::map::point::ECEFCoordinate valueQueryPointY(-6400000);
    valueQueryPoint.y = valueQueryPointY;
    ::ad::map::point::ECEFCoordinate valueQueryPointZ(-6400000);
    valueQueryPoint.z = valueQueryPointZ;
    value.queryPoint = valueQueryPoint;
    ::ad::physics::Distance valueMatchedPointDistance(-1e9);
    value.matchedPointDistance = valueMatchedPointDistance;
    mValue = value;
  }

  ::ad::map::match::MapMatchedPosition mValue;
};

TEST_F(MapMatchedPositionTests, copyConstruction)
{
  ::ad::map::match::MapMatchedPosition value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(MapMatchedPositionTests, moveConstruction)
{
  ::ad::map::match::MapMatchedPosition tmpValue(mValue);
  ::ad::map::match::MapMatchedPosition value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(MapMatchedPositionTests, copyAssignment)
{
  ::ad::map::match::MapMatchedPosition value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(MapMatchedPositionTests, moveAssignment)
{
  ::ad::map::match::MapMatchedPosition tmpValue(mValue);
  ::ad::map::match::MapMatchedPosition value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(MapMatchedPositionTests, comparisonOperatorEqual)
{
  ::ad::map::match::MapMatchedPosition valueA = mValue;
  ::ad::map::match::MapMatchedPosition valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(MapMatchedPositionTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(MapMatchedPositionTests, comparisonOperatorLanePointDiffers)
{
  ::ad::map::match::MapMatchedPosition valueA = mValue;
  ::ad::map::match::LanePoint lanePoint;
  ::ad::map::point::ParaPoint lanePointParaPoint;
  ::ad::map::lane::LaneId lanePointParaPointLaneId(std::numeric_limits<::ad::map::lane::LaneId>::max());
  lanePointParaPoint.laneId = lanePointParaPointLaneId;
  ::ad::physics::ParametricValue lanePointParaPointParametricOffset(1.);
  lanePointParaPoint.parametricOffset = lanePointParaPointParametricOffset;
  lanePoint.paraPoint = lanePointParaPoint;
  ::ad::physics::RatioValue lanePointLateralT(std::numeric_limits<::ad::physics::RatioValue>::max());
  lanePoint.lateralT = lanePointLateralT;
  ::ad::physics::Distance lanePointLaneLength(1e9);
  lanePoint.laneLength = lanePointLaneLength;
  ::ad::physics::Distance lanePointLaneWidth(1e9);
  lanePoint.laneWidth = lanePointLaneWidth;
  valueA.lanePoint = lanePoint;
  ::ad::map::match::MapMatchedPosition valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(MapMatchedPositionTests, comparisonOperatorTypeDiffers)
{
  ::ad::map::match::MapMatchedPosition valueA = mValue;
  ::ad::map::match::MapMatchedPositionType type(::ad::map::match::MapMatchedPositionType::LANE_RIGHT);
  valueA.type = type;
  ::ad::map::match::MapMatchedPosition valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(MapMatchedPositionTests, comparisonOperatorMatchedPointDiffers)
{
  ::ad::map::match::MapMatchedPosition valueA = mValue;
  ::ad::map::point::ECEFPoint matchedPoint;
  ::ad::map::point::ECEFCoordinate matchedPointX(6400000);
  matchedPoint.x = matchedPointX;
  ::ad::map::point::ECEFCoordinate matchedPointY(6400000);
  matchedPoint.y = matchedPointY;
  ::ad::map::point::ECEFCoordinate matchedPointZ(6400000);
  matchedPoint.z = matchedPointZ;
  valueA.matchedPoint = matchedPoint;
  ::ad::map::match::MapMatchedPosition valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(MapMatchedPositionTests, comparisonOperatorProbabilityDiffers)
{
  ::ad::map::match::MapMatchedPosition valueA = mValue;
  ::ad::physics::Probability probability(1.);
  valueA.probability = probability;
  ::ad::map::match::MapMatchedPosition valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(MapMatchedPositionTests, comparisonOperatorQueryPointDiffers)
{
  ::ad::map::match::MapMatchedPosition valueA = mValue;
  ::ad::map::point::ECEFPoint queryPoint;
  ::ad::map::point::ECEFCoordinate queryPointX(6400000);
  queryPoint.x = queryPointX;
  ::ad::map::point::ECEFCoordinate queryPointY(6400000);
  queryPoint.y = queryPointY;
  ::ad::map::point::ECEFCoordinate queryPointZ(6400000);
  queryPoint.z = queryPointZ;
  valueA.queryPoint = queryPoint;
  ::ad::map::match::MapMatchedPosition valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(MapMatchedPositionTests, comparisonOperatorMatchedPointDistanceDiffers)
{
  ::ad::map::match::MapMatchedPosition valueA = mValue;
  ::ad::physics::Distance matchedPointDistance(1e9);
  valueA.matchedPointDistance = matchedPointDistance;
  ::ad::map::match::MapMatchedPosition valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
