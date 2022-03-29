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

#include "ad/map/match/ObjectValidInputRange.hpp"

TEST(ObjectValidInputRangeTests, testValidInputRange)
{
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
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.x
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX;
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.y
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY;
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.z
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ;
  valueMapMatchedBoundingBoxReferencePointPositionsElementElement.queryPoint
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint;
  ::ad::physics::Distance valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance(-1e9);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElement.matchedPointDistance
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance;
  valueMapMatchedBoundingBoxReferencePointPositionsElement.resize(
    1, valueMapMatchedBoundingBoxReferencePointPositionsElementElement);
  valueMapMatchedBoundingBoxReferencePointPositions.resize(1, valueMapMatchedBoundingBoxReferencePointPositionsElement);
  valueMapMatchedBoundingBox.referencePointPositions = valueMapMatchedBoundingBoxReferencePointPositions;
  ::ad::physics::Distance valueMapMatchedBoundingBoxSamplingDistance(-1e9);
  valueMapMatchedBoundingBox.samplingDistance = valueMapMatchedBoundingBoxSamplingDistance;
  ::ad::physics::Distance valueMapMatchedBoundingBoxMatchRadius(-1e9);
  valueMapMatchedBoundingBox.matchRadius = valueMapMatchedBoundingBoxMatchRadius;
  value.mapMatchedBoundingBox = valueMapMatchedBoundingBox;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ObjectValidInputRangeTests, testValidInputRangeEnuPositionTooSmall)
{
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
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.x
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX;
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.y
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY;
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.z
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ;
  valueMapMatchedBoundingBoxReferencePointPositionsElementElement.queryPoint
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint;
  ::ad::physics::Distance valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance(-1e9);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElement.matchedPointDistance
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance;
  valueMapMatchedBoundingBoxReferencePointPositionsElement.resize(
    1, valueMapMatchedBoundingBoxReferencePointPositionsElementElement);
  valueMapMatchedBoundingBoxReferencePointPositions.resize(1, valueMapMatchedBoundingBoxReferencePointPositionsElement);
  valueMapMatchedBoundingBox.referencePointPositions = valueMapMatchedBoundingBoxReferencePointPositions;
  ::ad::physics::Distance valueMapMatchedBoundingBoxSamplingDistance(-1e9);
  valueMapMatchedBoundingBox.samplingDistance = valueMapMatchedBoundingBoxSamplingDistance;
  ::ad::physics::Distance valueMapMatchedBoundingBoxMatchRadius(-1e9);
  valueMapMatchedBoundingBox.matchRadius = valueMapMatchedBoundingBoxMatchRadius;
  value.mapMatchedBoundingBox = valueMapMatchedBoundingBox;

  // override member with data type value below input range minimum
  ::ad::map::match::ENUObjectPosition invalidInitializedMember;
  ::ad::map::point::ENUPoint invalidInitializedMemberCenterPoint;
  ::ad::map::point::ENUCoordinate invalidInitializedMemberCenterPointX(-16384 * 1.1);
  invalidInitializedMemberCenterPoint.x = invalidInitializedMemberCenterPointX;
  invalidInitializedMember.centerPoint = invalidInitializedMemberCenterPoint;
  value.enuPosition = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ObjectValidInputRangeTests, testValidInputRangeEnuPositionTooBig)
{
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
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.x
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX;
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.y
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY;
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.z
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ;
  valueMapMatchedBoundingBoxReferencePointPositionsElementElement.queryPoint
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint;
  ::ad::physics::Distance valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance(-1e9);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElement.matchedPointDistance
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance;
  valueMapMatchedBoundingBoxReferencePointPositionsElement.resize(
    1, valueMapMatchedBoundingBoxReferencePointPositionsElementElement);
  valueMapMatchedBoundingBoxReferencePointPositions.resize(1, valueMapMatchedBoundingBoxReferencePointPositionsElement);
  valueMapMatchedBoundingBox.referencePointPositions = valueMapMatchedBoundingBoxReferencePointPositions;
  ::ad::physics::Distance valueMapMatchedBoundingBoxSamplingDistance(-1e9);
  valueMapMatchedBoundingBox.samplingDistance = valueMapMatchedBoundingBoxSamplingDistance;
  ::ad::physics::Distance valueMapMatchedBoundingBoxMatchRadius(-1e9);
  valueMapMatchedBoundingBox.matchRadius = valueMapMatchedBoundingBoxMatchRadius;
  value.mapMatchedBoundingBox = valueMapMatchedBoundingBox;

  // override member with data type value above input range maximum
  ::ad::map::match::ENUObjectPosition invalidInitializedMember;
  ::ad::map::point::ENUPoint invalidInitializedMemberCenterPoint;
  ::ad::map::point::ENUCoordinate invalidInitializedMemberCenterPointX(16384 * 1.1);
  invalidInitializedMemberCenterPoint.x = invalidInitializedMemberCenterPointX;
  invalidInitializedMember.centerPoint = invalidInitializedMemberCenterPoint;
  value.enuPosition = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ObjectValidInputRangeTests, testValidInputRangeMapMatchedBoundingBoxTooSmall)
{
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
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.x
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX;
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.y
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY;
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.z
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ;
  valueMapMatchedBoundingBoxReferencePointPositionsElementElement.queryPoint
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint;
  ::ad::physics::Distance valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance(-1e9);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElement.matchedPointDistance
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance;
  valueMapMatchedBoundingBoxReferencePointPositionsElement.resize(
    1, valueMapMatchedBoundingBoxReferencePointPositionsElementElement);
  valueMapMatchedBoundingBoxReferencePointPositions.resize(1, valueMapMatchedBoundingBoxReferencePointPositionsElement);
  valueMapMatchedBoundingBox.referencePointPositions = valueMapMatchedBoundingBoxReferencePointPositions;
  ::ad::physics::Distance valueMapMatchedBoundingBoxSamplingDistance(-1e9);
  valueMapMatchedBoundingBox.samplingDistance = valueMapMatchedBoundingBoxSamplingDistance;
  ::ad::physics::Distance valueMapMatchedBoundingBoxMatchRadius(-1e9);
  valueMapMatchedBoundingBox.matchRadius = valueMapMatchedBoundingBoxMatchRadius;
  value.mapMatchedBoundingBox = valueMapMatchedBoundingBox;

  // override member with data type value below input range minimum
  ::ad::map::match::MapMatchedObjectBoundingBox invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberSamplingDistance(-1e9 * 1.1);
  invalidInitializedMember.samplingDistance = invalidInitializedMemberSamplingDistance;
  value.mapMatchedBoundingBox = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ObjectValidInputRangeTests, testValidInputRangeMapMatchedBoundingBoxTooBig)
{
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
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.x
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointX;
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.y
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointY;
  ::ad::map::point::ECEFCoordinate valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ(-6400000);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint.z
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPointZ;
  valueMapMatchedBoundingBoxReferencePointPositionsElementElement.queryPoint
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementQueryPoint;
  ::ad::physics::Distance valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance(-1e9);
  valueMapMatchedBoundingBoxReferencePointPositionsElementElement.matchedPointDistance
    = valueMapMatchedBoundingBoxReferencePointPositionsElementElementMatchedPointDistance;
  valueMapMatchedBoundingBoxReferencePointPositionsElement.resize(
    1, valueMapMatchedBoundingBoxReferencePointPositionsElementElement);
  valueMapMatchedBoundingBoxReferencePointPositions.resize(1, valueMapMatchedBoundingBoxReferencePointPositionsElement);
  valueMapMatchedBoundingBox.referencePointPositions = valueMapMatchedBoundingBoxReferencePointPositions;
  ::ad::physics::Distance valueMapMatchedBoundingBoxSamplingDistance(-1e9);
  valueMapMatchedBoundingBox.samplingDistance = valueMapMatchedBoundingBoxSamplingDistance;
  ::ad::physics::Distance valueMapMatchedBoundingBoxMatchRadius(-1e9);
  valueMapMatchedBoundingBox.matchRadius = valueMapMatchedBoundingBoxMatchRadius;
  value.mapMatchedBoundingBox = valueMapMatchedBoundingBox;

  // override member with data type value above input range maximum
  ::ad::map::match::MapMatchedObjectBoundingBox invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberSamplingDistance(1e9 * 1.1);
  invalidInitializedMember.samplingDistance = invalidInitializedMemberSamplingDistance;
  value.mapMatchedBoundingBox = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
