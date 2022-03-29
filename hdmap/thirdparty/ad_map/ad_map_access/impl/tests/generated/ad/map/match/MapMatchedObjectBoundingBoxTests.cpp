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
#include "ad/map/match/MapMatchedObjectBoundingBox.hpp"

class MapMatchedObjectBoundingBoxTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::match::MapMatchedObjectBoundingBox value;
    ::ad::map::match::LaneOccupiedRegionList valueLaneOccupiedRegions;
    ::ad::map::match::LaneOccupiedRegion valueLaneOccupiedRegionsElement;
    ::ad::map::lane::LaneId valueLaneOccupiedRegionsElementLaneId(1);
    valueLaneOccupiedRegionsElement.laneId = valueLaneOccupiedRegionsElementLaneId;
    ::ad::physics::ParametricRange valueLaneOccupiedRegionsElementLongitudinalRange;
    ::ad::physics::ParametricValue valueLaneOccupiedRegionsElementLongitudinalRangeMinimum(0.);
    valueLaneOccupiedRegionsElementLongitudinalRange.minimum = valueLaneOccupiedRegionsElementLongitudinalRangeMinimum;
    ::ad::physics::ParametricValue valueLaneOccupiedRegionsElementLongitudinalRangeMaximum(0.);
    valueLaneOccupiedRegionsElementLongitudinalRange.maximum = valueLaneOccupiedRegionsElementLongitudinalRangeMaximum;
    valueLaneOccupiedRegionsElementLongitudinalRange.maximum = valueLaneOccupiedRegionsElementLongitudinalRange.minimum;
    valueLaneOccupiedRegionsElementLongitudinalRange.minimum = valueLaneOccupiedRegionsElementLongitudinalRange.maximum;
    valueLaneOccupiedRegionsElement.longitudinalRange = valueLaneOccupiedRegionsElementLongitudinalRange;
    ::ad::physics::ParametricRange valueLaneOccupiedRegionsElementLateralRange;
    ::ad::physics::ParametricValue valueLaneOccupiedRegionsElementLateralRangeMinimum(0.);
    valueLaneOccupiedRegionsElementLateralRange.minimum = valueLaneOccupiedRegionsElementLateralRangeMinimum;
    ::ad::physics::ParametricValue valueLaneOccupiedRegionsElementLateralRangeMaximum(0.);
    valueLaneOccupiedRegionsElementLateralRange.maximum = valueLaneOccupiedRegionsElementLateralRangeMaximum;
    valueLaneOccupiedRegionsElementLateralRange.maximum = valueLaneOccupiedRegionsElementLateralRange.minimum;
    valueLaneOccupiedRegionsElementLateralRange.minimum = valueLaneOccupiedRegionsElementLateralRange.maximum;
    valueLaneOccupiedRegionsElement.lateralRange = valueLaneOccupiedRegionsElementLateralRange;
    valueLaneOccupiedRegions.resize(1, valueLaneOccupiedRegionsElement);
    value.laneOccupiedRegions = valueLaneOccupiedRegions;
    ::ad::map::match::MapMatchedObjectReferencePositionList valueReferencePointPositions;
    ::ad::map::match::MapMatchedPositionConfidenceList valueReferencePointPositionsElement;
    ::ad::map::match::MapMatchedPosition valueReferencePointPositionsElementElement;
    ::ad::map::match::LanePoint valueReferencePointPositionsElementElementLanePoint;
    ::ad::map::point::ParaPoint valueReferencePointPositionsElementElementLanePointParaPoint;
    ::ad::map::lane::LaneId valueReferencePointPositionsElementElementLanePointParaPointLaneId(1);
    valueReferencePointPositionsElementElementLanePointParaPoint.laneId
      = valueReferencePointPositionsElementElementLanePointParaPointLaneId;
    ::ad::physics::ParametricValue valueReferencePointPositionsElementElementLanePointParaPointParametricOffset(0.);
    valueReferencePointPositionsElementElementLanePointParaPoint.parametricOffset
      = valueReferencePointPositionsElementElementLanePointParaPointParametricOffset;
    valueReferencePointPositionsElementElementLanePoint.paraPoint
      = valueReferencePointPositionsElementElementLanePointParaPoint;
    ::ad::physics::RatioValue valueReferencePointPositionsElementElementLanePointLateralT(
      std::numeric_limits<::ad::physics::RatioValue>::lowest());
    valueReferencePointPositionsElementElementLanePoint.lateralT
      = valueReferencePointPositionsElementElementLanePointLateralT;
    ::ad::physics::Distance valueReferencePointPositionsElementElementLanePointLaneLength(-1e9);
    valueReferencePointPositionsElementElementLanePoint.laneLength
      = valueReferencePointPositionsElementElementLanePointLaneLength;
    ::ad::physics::Distance valueReferencePointPositionsElementElementLanePointLaneWidth(-1e9);
    valueReferencePointPositionsElementElementLanePoint.laneWidth
      = valueReferencePointPositionsElementElementLanePointLaneWidth;
    valueReferencePointPositionsElementElement.lanePoint = valueReferencePointPositionsElementElementLanePoint;
    ::ad::map::match::MapMatchedPositionType valueReferencePointPositionsElementElementType(
      ::ad::map::match::MapMatchedPositionType::INVALID);
    valueReferencePointPositionsElementElement.type = valueReferencePointPositionsElementElementType;
    ::ad::map::point::ECEFPoint valueReferencePointPositionsElementElementMatchedPoint;
    ::ad::map::point::ECEFCoordinate valueReferencePointPositionsElementElementMatchedPointX(-6400000);
    valueReferencePointPositionsElementElementMatchedPoint.x = valueReferencePointPositionsElementElementMatchedPointX;
    ::ad::map::point::ECEFCoordinate valueReferencePointPositionsElementElementMatchedPointY(-6400000);
    valueReferencePointPositionsElementElementMatchedPoint.y = valueReferencePointPositionsElementElementMatchedPointY;
    ::ad::map::point::ECEFCoordinate valueReferencePointPositionsElementElementMatchedPointZ(-6400000);
    valueReferencePointPositionsElementElementMatchedPoint.z = valueReferencePointPositionsElementElementMatchedPointZ;
    valueReferencePointPositionsElementElement.matchedPoint = valueReferencePointPositionsElementElementMatchedPoint;
    ::ad::physics::Probability valueReferencePointPositionsElementElementProbability(0.);
    valueReferencePointPositionsElementElement.probability = valueReferencePointPositionsElementElementProbability;
    ::ad::map::point::ECEFPoint valueReferencePointPositionsElementElementQueryPoint;
    ::ad::map::point::ECEFCoordinate valueReferencePointPositionsElementElementQueryPointX(-6400000);
    valueReferencePointPositionsElementElementQueryPoint.x = valueReferencePointPositionsElementElementQueryPointX;
    ::ad::map::point::ECEFCoordinate valueReferencePointPositionsElementElementQueryPointY(-6400000);
    valueReferencePointPositionsElementElementQueryPoint.y = valueReferencePointPositionsElementElementQueryPointY;
    ::ad::map::point::ECEFCoordinate valueReferencePointPositionsElementElementQueryPointZ(-6400000);
    valueReferencePointPositionsElementElementQueryPoint.z = valueReferencePointPositionsElementElementQueryPointZ;
    valueReferencePointPositionsElementElement.queryPoint = valueReferencePointPositionsElementElementQueryPoint;
    ::ad::physics::Distance valueReferencePointPositionsElementElementMatchedPointDistance(-1e9);
    valueReferencePointPositionsElementElement.matchedPointDistance
      = valueReferencePointPositionsElementElementMatchedPointDistance;
    valueReferencePointPositionsElement.resize(1, valueReferencePointPositionsElementElement);
    valueReferencePointPositions.resize(1, valueReferencePointPositionsElement);
    value.referencePointPositions = valueReferencePointPositions;
    ::ad::physics::Distance valueSamplingDistance(-1e9);
    value.samplingDistance = valueSamplingDistance;
    ::ad::physics::Distance valueMatchRadius(-1e9);
    value.matchRadius = valueMatchRadius;
    mValue = value;
  }

  ::ad::map::match::MapMatchedObjectBoundingBox mValue;
};

TEST_F(MapMatchedObjectBoundingBoxTests, copyConstruction)
{
  ::ad::map::match::MapMatchedObjectBoundingBox value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(MapMatchedObjectBoundingBoxTests, moveConstruction)
{
  ::ad::map::match::MapMatchedObjectBoundingBox tmpValue(mValue);
  ::ad::map::match::MapMatchedObjectBoundingBox value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(MapMatchedObjectBoundingBoxTests, copyAssignment)
{
  ::ad::map::match::MapMatchedObjectBoundingBox value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(MapMatchedObjectBoundingBoxTests, moveAssignment)
{
  ::ad::map::match::MapMatchedObjectBoundingBox tmpValue(mValue);
  ::ad::map::match::MapMatchedObjectBoundingBox value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(MapMatchedObjectBoundingBoxTests, comparisonOperatorEqual)
{
  ::ad::map::match::MapMatchedObjectBoundingBox valueA = mValue;
  ::ad::map::match::MapMatchedObjectBoundingBox valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(MapMatchedObjectBoundingBoxTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(MapMatchedObjectBoundingBoxTests, comparisonOperatorLaneOccupiedRegionsDiffers)
{
  ::ad::map::match::MapMatchedObjectBoundingBox valueA = mValue;
  ::ad::map::match::LaneOccupiedRegionList laneOccupiedRegions;
  ::ad::map::match::LaneOccupiedRegion laneOccupiedRegionsElement;
  ::ad::map::lane::LaneId laneOccupiedRegionsElementLaneId(std::numeric_limits<::ad::map::lane::LaneId>::max());
  laneOccupiedRegionsElement.laneId = laneOccupiedRegionsElementLaneId;
  ::ad::physics::ParametricRange laneOccupiedRegionsElementLongitudinalRange;
  ::ad::physics::ParametricValue laneOccupiedRegionsElementLongitudinalRangeMinimum(1.);
  laneOccupiedRegionsElementLongitudinalRange.minimum = laneOccupiedRegionsElementLongitudinalRangeMinimum;
  ::ad::physics::ParametricValue laneOccupiedRegionsElementLongitudinalRangeMaximum(1.);
  laneOccupiedRegionsElementLongitudinalRange.maximum = laneOccupiedRegionsElementLongitudinalRangeMaximum;
  laneOccupiedRegionsElementLongitudinalRange.maximum = laneOccupiedRegionsElementLongitudinalRange.minimum;
  laneOccupiedRegionsElementLongitudinalRange.minimum = laneOccupiedRegionsElementLongitudinalRange.maximum;
  laneOccupiedRegionsElement.longitudinalRange = laneOccupiedRegionsElementLongitudinalRange;
  ::ad::physics::ParametricRange laneOccupiedRegionsElementLateralRange;
  ::ad::physics::ParametricValue laneOccupiedRegionsElementLateralRangeMinimum(1.);
  laneOccupiedRegionsElementLateralRange.minimum = laneOccupiedRegionsElementLateralRangeMinimum;
  ::ad::physics::ParametricValue laneOccupiedRegionsElementLateralRangeMaximum(1.);
  laneOccupiedRegionsElementLateralRange.maximum = laneOccupiedRegionsElementLateralRangeMaximum;
  laneOccupiedRegionsElementLateralRange.maximum = laneOccupiedRegionsElementLateralRange.minimum;
  laneOccupiedRegionsElementLateralRange.minimum = laneOccupiedRegionsElementLateralRange.maximum;
  laneOccupiedRegionsElement.lateralRange = laneOccupiedRegionsElementLateralRange;
  laneOccupiedRegions.resize(2, laneOccupiedRegionsElement);
  valueA.laneOccupiedRegions = laneOccupiedRegions;
  ::ad::map::match::MapMatchedObjectBoundingBox valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(MapMatchedObjectBoundingBoxTests, comparisonOperatorReferencePointPositionsDiffers)
{
  ::ad::map::match::MapMatchedObjectBoundingBox valueA = mValue;
  ::ad::map::match::MapMatchedObjectReferencePositionList referencePointPositions;
  ::ad::map::match::MapMatchedPositionConfidenceList referencePointPositionsElement;
  ::ad::map::match::MapMatchedPosition referencePointPositionsElementElement;
  ::ad::map::match::LanePoint referencePointPositionsElementElementLanePoint;
  ::ad::map::point::ParaPoint referencePointPositionsElementElementLanePointParaPoint;
  ::ad::map::lane::LaneId referencePointPositionsElementElementLanePointParaPointLaneId(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  referencePointPositionsElementElementLanePointParaPoint.laneId
    = referencePointPositionsElementElementLanePointParaPointLaneId;
  ::ad::physics::ParametricValue referencePointPositionsElementElementLanePointParaPointParametricOffset(1.);
  referencePointPositionsElementElementLanePointParaPoint.parametricOffset
    = referencePointPositionsElementElementLanePointParaPointParametricOffset;
  referencePointPositionsElementElementLanePoint.paraPoint = referencePointPositionsElementElementLanePointParaPoint;
  ::ad::physics::RatioValue referencePointPositionsElementElementLanePointLateralT(
    std::numeric_limits<::ad::physics::RatioValue>::max());
  referencePointPositionsElementElementLanePoint.lateralT = referencePointPositionsElementElementLanePointLateralT;
  ::ad::physics::Distance referencePointPositionsElementElementLanePointLaneLength(1e9);
  referencePointPositionsElementElementLanePoint.laneLength = referencePointPositionsElementElementLanePointLaneLength;
  ::ad::physics::Distance referencePointPositionsElementElementLanePointLaneWidth(1e9);
  referencePointPositionsElementElementLanePoint.laneWidth = referencePointPositionsElementElementLanePointLaneWidth;
  referencePointPositionsElementElement.lanePoint = referencePointPositionsElementElementLanePoint;
  ::ad::map::match::MapMatchedPositionType referencePointPositionsElementElementType(
    ::ad::map::match::MapMatchedPositionType::LANE_RIGHT);
  referencePointPositionsElementElement.type = referencePointPositionsElementElementType;
  ::ad::map::point::ECEFPoint referencePointPositionsElementElementMatchedPoint;
  ::ad::map::point::ECEFCoordinate referencePointPositionsElementElementMatchedPointX(6400000);
  referencePointPositionsElementElementMatchedPoint.x = referencePointPositionsElementElementMatchedPointX;
  ::ad::map::point::ECEFCoordinate referencePointPositionsElementElementMatchedPointY(6400000);
  referencePointPositionsElementElementMatchedPoint.y = referencePointPositionsElementElementMatchedPointY;
  ::ad::map::point::ECEFCoordinate referencePointPositionsElementElementMatchedPointZ(6400000);
  referencePointPositionsElementElementMatchedPoint.z = referencePointPositionsElementElementMatchedPointZ;
  referencePointPositionsElementElement.matchedPoint = referencePointPositionsElementElementMatchedPoint;
  ::ad::physics::Probability referencePointPositionsElementElementProbability(1.);
  referencePointPositionsElementElement.probability = referencePointPositionsElementElementProbability;
  ::ad::map::point::ECEFPoint referencePointPositionsElementElementQueryPoint;
  ::ad::map::point::ECEFCoordinate referencePointPositionsElementElementQueryPointX(6400000);
  referencePointPositionsElementElementQueryPoint.x = referencePointPositionsElementElementQueryPointX;
  ::ad::map::point::ECEFCoordinate referencePointPositionsElementElementQueryPointY(6400000);
  referencePointPositionsElementElementQueryPoint.y = referencePointPositionsElementElementQueryPointY;
  ::ad::map::point::ECEFCoordinate referencePointPositionsElementElementQueryPointZ(6400000);
  referencePointPositionsElementElementQueryPoint.z = referencePointPositionsElementElementQueryPointZ;
  referencePointPositionsElementElement.queryPoint = referencePointPositionsElementElementQueryPoint;
  ::ad::physics::Distance referencePointPositionsElementElementMatchedPointDistance(1e9);
  referencePointPositionsElementElement.matchedPointDistance
    = referencePointPositionsElementElementMatchedPointDistance;
  referencePointPositionsElement.resize(2, referencePointPositionsElementElement);
  referencePointPositions.resize(2, referencePointPositionsElement);
  valueA.referencePointPositions = referencePointPositions;
  ::ad::map::match::MapMatchedObjectBoundingBox valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(MapMatchedObjectBoundingBoxTests, comparisonOperatorSamplingDistanceDiffers)
{
  ::ad::map::match::MapMatchedObjectBoundingBox valueA = mValue;
  ::ad::physics::Distance samplingDistance(1e9);
  valueA.samplingDistance = samplingDistance;
  ::ad::map::match::MapMatchedObjectBoundingBox valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(MapMatchedObjectBoundingBoxTests, comparisonOperatorMatchRadiusDiffers)
{
  ::ad::map::match::MapMatchedObjectBoundingBox valueA = mValue;
  ::ad::physics::Distance matchRadius(1e9);
  valueA.matchRadius = matchRadius;
  ::ad::map::match::MapMatchedObjectBoundingBox valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
