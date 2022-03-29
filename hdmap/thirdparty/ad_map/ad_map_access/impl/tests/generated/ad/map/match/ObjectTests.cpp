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
#include "ad/map/match/Object.hpp"

class ObjectTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::match::Object value;
    ::ad::map::match::ENUObjectPosition valueEnuPosition;
    ::ad::map::point::ENUPoint valueEnuPositionCenterPoint;
    ::ad::map::point::ENUCoordinate valueEnuPositionCenterPointX(-16384);
    valueEnuPositionCenterPoint.x = valueEnuPositionCenterPointX;
    ::ad::map::point::ENUCoordinate valueEnuPositionCenterPointY(-16384);
    valueEnuPositionCenterPoint.y = valueEnuPositionCenterPointY;
    ::ad::map::point::ENUCoordinate valueEnuPositionCenterPointZ(-16384);
    valueEnuPositionCenterPoint.z = valueEnuPositionCenterPointZ;
    valueEnuPosition.centerPoint = valueEnuPositionCenterPoint;
    ::ad::map::point::ENUHeading valueEnuPositionHeading(-3.141592655);
    valueEnuPosition.heading = valueEnuPositionHeading;
    ::ad::map::point::GeoPoint valueEnuPositionEnuReferencePoint;
    ::ad::map::point::Longitude valueEnuPositionEnuReferencePointLongitude(-180);
    valueEnuPositionEnuReferencePoint.longitude = valueEnuPositionEnuReferencePointLongitude;
    ::ad::map::point::Latitude valueEnuPositionEnuReferencePointLatitude(-90);
    valueEnuPositionEnuReferencePoint.latitude = valueEnuPositionEnuReferencePointLatitude;
    ::ad::map::point::Altitude valueEnuPositionEnuReferencePointAltitude(-11000);
    valueEnuPositionEnuReferencePoint.altitude = valueEnuPositionEnuReferencePointAltitude;
    valueEnuPosition.enuReferencePoint = valueEnuPositionEnuReferencePoint;
    ::ad::physics::Dimension3D valueEnuPositionDimension;
    ::ad::physics::Distance valueEnuPositionDimensionLength(-1e9);
    valueEnuPositionDimension.length = valueEnuPositionDimensionLength;
    ::ad::physics::Distance valueEnuPositionDimensionWidth(-1e9);
    valueEnuPositionDimension.width = valueEnuPositionDimensionWidth;
    ::ad::physics::Distance valueEnuPositionDimensionHeight(-1e9);
    valueEnuPositionDimension.height = valueEnuPositionDimensionHeight;
    valueEnuPosition.dimension = valueEnuPositionDimension;
    value.enuPosition = valueEnuPosition;
    ::ad::map::match::MapMatchedObjectBoundingBox valueMapMatchedBoundingBox;
    ::ad::map::match::LaneOccupiedRegionList valueMapMatchedBoundingBoxLaneOccupiedRegions;
    ::ad::map::match::LaneOccupiedRegion valueMapMatchedBoundingBoxLaneOccupiedRegionsElement;
    ::ad::map::lane::LaneId valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLaneId(1);
    valueMapMatchedBoundingBoxLaneOccupiedRegionsElement.laneId
      = valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLaneId;
    ::ad::physics::ParametricRange valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange;
    ::ad::physics::ParametricValue valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRangeMinimum(0.);
    valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange.minimum
      = valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRangeMinimum;
    ::ad::physics::ParametricValue valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRangeMaximum(0.);
    valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange.maximum
      = valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRangeMaximum;
    valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange.maximum
      = valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange.minimum;
    valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange.minimum
      = valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange.maximum;
    valueMapMatchedBoundingBoxLaneOccupiedRegionsElement.longitudinalRange
      = valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange;
    ::ad::physics::ParametricRange valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange;
    ::ad::physics::ParametricValue valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRangeMinimum(0.);
    valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange.minimum
      = valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRangeMinimum;
    ::ad::physics::ParametricValue valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRangeMaximum(0.);
    valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange.maximum
      = valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRangeMaximum;
    valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange.maximum
      = valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange.minimum;
    valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange.minimum
      = valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange.maximum;
    valueMapMatchedBoundingBoxLaneOccupiedRegionsElement.lateralRange
      = valueMapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange;
    valueMapMatchedBoundingBoxLaneOccupiedRegions.resize(1, valueMapMatchedBoundingBoxLaneOccupiedRegionsElement);
    valueMapMatchedBoundingBox.laneOccupiedRegions = valueMapMatchedBoundingBoxLaneOccupiedRegions;
    ::ad::map::match::MapMatchedObjectReferencePositionList valueMapMatchedBoundingBoxReferencePointPositions;
    ::ad::map::match::MapMatchedPositionConfidenceList valueMapMatchedBoundingBoxReferencePointPositionsElement;
    ::ad::map::match::MapMatchedPosition valueMapMatchedBoundingBoxReferencePointPositionsElementElement;
    ::ad::map::match::LanePoint valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePoint;
    ::ad::map::point::ParaPoint valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPoint;
    ::ad::map::lane::LaneId valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPointLaneId(1);
    valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPoint.laneId
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPointLaneId;
    ::ad::physics::ParametricValue
      valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPointParametricOffset(0.);
    valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPoint.parametricOffset
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPointParametricOffset;
    valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePoint.paraPoint
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPoint;
    ::ad::physics::RatioValue valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointLateralT(
      std::numeric_limits<::ad::physics::RatioValue>::lowest());
    valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePoint.lateralT
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointLateralT;
    ::ad::physics::Distance valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointLaneLength(-1e9);
    valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePoint.laneLength
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointLaneLength;
    ::ad::physics::Distance valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointLaneWidth(-1e9);
    valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePoint.laneWidth
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePointLaneWidth;
    valueMapMatchedBoundingBoxReferencePointPositionsElementElement.lanePoint
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementLanePoint;
    ::ad::map::match::MapMatchedPositionType valueMapMatchedBoundingBoxReferencePointPositionsElementElementType(
      ::ad::map::match::MapMatchedPositionType::INVALID);
    valueMapMatchedBoundingBoxReferencePointPositionsElementElement.type
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementType;
    ::ad::map::point::ECEFPoint valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPoint;
    ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointX(
      -6400000);
    valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPoint.x
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointX;
    ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointY(
      -6400000);
    valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPoint.y
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointY;
    ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointZ(
      -6400000);
    valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPoint.z
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointZ;
    valueMapMatchedBoundingBoxReferencePointPositionsElementElement.matchedPoint
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPoint;
    ::ad::physics::Probability valueMapMatchedBoundingBoxReferencePointPositionsElementElementProbability(0.);
    valueMapMatchedBoundingBoxReferencePointPositionsElementElement.probability
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementProbability;
    ::ad::map::point::ECEFPoint valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint;
    ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX(
      -6400000);
    valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.x
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX;
    ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY(
      -6400000);
    valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.y
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY;
    ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ(
      -6400000);
    valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.z
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ;
    valueMapMatchedBoundingBoxReferencePointPositionsElementElement.queryPoint
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint;
    ::ad::physics::Distance valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance(-1e9);
    valueMapMatchedBoundingBoxReferencePointPositionsElementElement.matchedPointDistance
      = valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance;
    valueMapMatchedBoundingBoxReferencePointPositionsElement.resize(
      1, valueMapMatchedBoundingBoxReferencePointPositionsElementElement);
    valueMapMatchedBoundingBoxReferencePointPositions.resize(1,
                                                             valueMapMatchedBoundingBoxReferencePointPositionsElement);
    valueMapMatchedBoundingBox.referencePointPositions = valueMapMatchedBoundingBoxReferencePointPositions;
    ::ad::physics::Distance valueMapMatchedBoundingBoxSamplingDistance(-1e9);
    valueMapMatchedBoundingBox.samplingDistance = valueMapMatchedBoundingBoxSamplingDistance;
    ::ad::physics::Distance valueMapMatchedBoundingBoxMatchRadius(-1e9);
    valueMapMatchedBoundingBox.matchRadius = valueMapMatchedBoundingBoxMatchRadius;
    value.mapMatchedBoundingBox = valueMapMatchedBoundingBox;
    mValue = value;
  }

  ::ad::map::match::Object mValue;
};

TEST_F(ObjectTests, copyConstruction)
{
  ::ad::map::match::Object value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ObjectTests, moveConstruction)
{
  ::ad::map::match::Object tmpValue(mValue);
  ::ad::map::match::Object value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(ObjectTests, copyAssignment)
{
  ::ad::map::match::Object value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(ObjectTests, moveAssignment)
{
  ::ad::map::match::Object tmpValue(mValue);
  ::ad::map::match::Object value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ObjectTests, comparisonOperatorEqual)
{
  ::ad::map::match::Object valueA = mValue;
  ::ad::map::match::Object valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(ObjectTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(ObjectTests, comparisonOperatorEnuPositionDiffers)
{
  ::ad::map::match::Object valueA = mValue;
  ::ad::map::match::ENUObjectPosition enuPosition;
  ::ad::map::point::ENUPoint enuPositionCenterPoint;
  ::ad::map::point::ENUCoordinate enuPositionCenterPointX(16384);
  enuPositionCenterPoint.x = enuPositionCenterPointX;
  ::ad::map::point::ENUCoordinate enuPositionCenterPointY(16384);
  enuPositionCenterPoint.y = enuPositionCenterPointY;
  ::ad::map::point::ENUCoordinate enuPositionCenterPointZ(16384);
  enuPositionCenterPoint.z = enuPositionCenterPointZ;
  enuPosition.centerPoint = enuPositionCenterPoint;
  ::ad::map::point::ENUHeading enuPositionHeading(3.141592655);
  enuPosition.heading = enuPositionHeading;
  ::ad::map::point::GeoPoint enuPositionEnuReferencePoint;
  ::ad::map::point::Longitude enuPositionEnuReferencePointLongitude(180);
  enuPositionEnuReferencePoint.longitude = enuPositionEnuReferencePointLongitude;
  ::ad::map::point::Latitude enuPositionEnuReferencePointLatitude(90);
  enuPositionEnuReferencePoint.latitude = enuPositionEnuReferencePointLatitude;
  ::ad::map::point::Altitude enuPositionEnuReferencePointAltitude(9000);
  enuPositionEnuReferencePoint.altitude = enuPositionEnuReferencePointAltitude;
  enuPosition.enuReferencePoint = enuPositionEnuReferencePoint;
  ::ad::physics::Dimension3D enuPositionDimension;
  ::ad::physics::Distance enuPositionDimensionLength(1e9);
  enuPositionDimension.length = enuPositionDimensionLength;
  ::ad::physics::Distance enuPositionDimensionWidth(1e9);
  enuPositionDimension.width = enuPositionDimensionWidth;
  ::ad::physics::Distance enuPositionDimensionHeight(1e9);
  enuPositionDimension.height = enuPositionDimensionHeight;
  enuPosition.dimension = enuPositionDimension;
  valueA.enuPosition = enuPosition;
  ::ad::map::match::Object valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ObjectTests, comparisonOperatorMapMatchedBoundingBoxDiffers)
{
  ::ad::map::match::Object valueA = mValue;
  ::ad::map::match::MapMatchedObjectBoundingBox mapMatchedBoundingBox;
  ::ad::map::match::LaneOccupiedRegionList mapMatchedBoundingBoxLaneOccupiedRegions;
  ::ad::map::match::LaneOccupiedRegion mapMatchedBoundingBoxLaneOccupiedRegionsElement;
  ::ad::map::lane::LaneId mapMatchedBoundingBoxLaneOccupiedRegionsElementLaneId(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  mapMatchedBoundingBoxLaneOccupiedRegionsElement.laneId = mapMatchedBoundingBoxLaneOccupiedRegionsElementLaneId;
  ::ad::physics::ParametricRange mapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange;
  ::ad::physics::ParametricValue mapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRangeMinimum(1.);
  mapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange.minimum
    = mapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRangeMinimum;
  ::ad::physics::ParametricValue mapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRangeMaximum(1.);
  mapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange.maximum
    = mapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRangeMaximum;
  mapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange.maximum
    = mapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange.minimum;
  mapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange.minimum
    = mapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange.maximum;
  mapMatchedBoundingBoxLaneOccupiedRegionsElement.longitudinalRange
    = mapMatchedBoundingBoxLaneOccupiedRegionsElementLongitudinalRange;
  ::ad::physics::ParametricRange mapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange;
  ::ad::physics::ParametricValue mapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRangeMinimum(1.);
  mapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange.minimum
    = mapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRangeMinimum;
  ::ad::physics::ParametricValue mapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRangeMaximum(1.);
  mapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange.maximum
    = mapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRangeMaximum;
  mapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange.maximum
    = mapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange.minimum;
  mapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange.minimum
    = mapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange.maximum;
  mapMatchedBoundingBoxLaneOccupiedRegionsElement.lateralRange
    = mapMatchedBoundingBoxLaneOccupiedRegionsElementLateralRange;
  mapMatchedBoundingBoxLaneOccupiedRegions.resize(2, mapMatchedBoundingBoxLaneOccupiedRegionsElement);
  mapMatchedBoundingBox.laneOccupiedRegions = mapMatchedBoundingBoxLaneOccupiedRegions;
  ::ad::map::match::MapMatchedObjectReferencePositionList mapMatchedBoundingBoxReferencePointPositions;
  ::ad::map::match::MapMatchedPositionConfidenceList mapMatchedBoundingBoxReferencePointPositionsElement;
  ::ad::map::match::MapMatchedPosition mapMatchedBoundingBoxReferencePointPositionsElementElement;
  ::ad::map::match::LanePoint mapMatchedBoundingBoxReferencePointPositionsElementElementLanePoint;
  ::ad::map::point::ParaPoint mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPoint;
  ::ad::map::lane::LaneId mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPointLaneId(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPoint.laneId
    = mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPointLaneId;
  ::ad::physics::ParametricValue
    mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPointParametricOffset(1.);
  mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPoint.parametricOffset
    = mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPointParametricOffset;
  mapMatchedBoundingBoxReferencePointPositionsElementElementLanePoint.paraPoint
    = mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointParaPoint;
  ::ad::physics::RatioValue mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointLateralT(
    std::numeric_limits<::ad::physics::RatioValue>::max());
  mapMatchedBoundingBoxReferencePointPositionsElementElementLanePoint.lateralT
    = mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointLateralT;
  ::ad::physics::Distance mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointLaneLength(1e9);
  mapMatchedBoundingBoxReferencePointPositionsElementElementLanePoint.laneLength
    = mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointLaneLength;
  ::ad::physics::Distance mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointLaneWidth(1e9);
  mapMatchedBoundingBoxReferencePointPositionsElementElementLanePoint.laneWidth
    = mapMatchedBoundingBoxReferencePointPositionsElementElementLanePointLaneWidth;
  mapMatchedBoundingBoxReferencePointPositionsElementElement.lanePoint
    = mapMatchedBoundingBoxReferencePointPositionsElementElementLanePoint;
  ::ad::map::match::MapMatchedPositionType mapMatchedBoundingBoxReferencePointPositionsElementElementType(
    ::ad::map::match::MapMatchedPositionType::LANE_RIGHT);
  mapMatchedBoundingBoxReferencePointPositionsElementElement.type
    = mapMatchedBoundingBoxReferencePointPositionsElementElementType;
  ::ad::map::point::ECEFPoint mapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPoint;
  ::ad::map::point::ECEFCoordinate mapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointX(6400000);
  mapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPoint.x
    = mapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointX;
  ::ad::map::point::ECEFCoordinate mapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointY(6400000);
  mapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPoint.y
    = mapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointY;
  ::ad::map::point::ECEFCoordinate mapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointZ(6400000);
  mapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPoint.z
    = mapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointZ;
  mapMatchedBoundingBoxReferencePointPositionsElementElement.matchedPoint
    = mapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPoint;
  ::ad::physics::Probability mapMatchedBoundingBoxReferencePointPositionsElementElementProbability(1.);
  mapMatchedBoundingBoxReferencePointPositionsElementElement.probability
    = mapMatchedBoundingBoxReferencePointPositionsElementElementProbability;
  ::ad::map::point::ECEFPoint mapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint;
  ::ad::map::point::ECEFCoordinate mapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX(6400000);
  mapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.x
    = mapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX;
  ::ad::map::point::ECEFCoordinate mapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY(6400000);
  mapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.y
    = mapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY;
  ::ad::map::point::ECEFCoordinate mapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ(6400000);
  mapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.z
    = mapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ;
  mapMatchedBoundingBoxReferencePointPositionsElementElement.queryPoint
    = mapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint;
  ::ad::physics::Distance mapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance(1e9);
  mapMatchedBoundingBoxReferencePointPositionsElementElement.matchedPointDistance
    = mapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance;
  mapMatchedBoundingBoxReferencePointPositionsElement.resize(
    2, mapMatchedBoundingBoxReferencePointPositionsElementElement);
  mapMatchedBoundingBoxReferencePointPositions.resize(2, mapMatchedBoundingBoxReferencePointPositionsElement);
  mapMatchedBoundingBox.referencePointPositions = mapMatchedBoundingBoxReferencePointPositions;
  ::ad::physics::Distance mapMatchedBoundingBoxSamplingDistance(1e9);
  mapMatchedBoundingBox.samplingDistance = mapMatchedBoundingBoxSamplingDistance;
  ::ad::physics::Distance mapMatchedBoundingBoxMatchRadius(1e9);
  mapMatchedBoundingBox.matchRadius = mapMatchedBoundingBoxMatchRadius;
  valueA.mapMatchedBoundingBox = mapMatchedBoundingBox;
  ::ad::map::match::Object valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
