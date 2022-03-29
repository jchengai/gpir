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
#include "ad/map/lane/Lane.hpp"

class LaneTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::lane::Lane value;
    ::ad::map::lane::LaneId valueId(1);
    value.id = valueId;
    ::ad::map::lane::LaneType valueType(::ad::map::lane::LaneType::INVALID);
    value.type = valueType;
    ::ad::map::lane::LaneDirection valueDirection(::ad::map::lane::LaneDirection::INVALID);
    value.direction = valueDirection;
    ::ad::map::restriction::Restrictions valueRestrictions;
    ::ad::map::restriction::RestrictionList valueRestrictionsConjunctions;
    ::ad::map::restriction::Restriction valueRestrictionsConjunctionsElement;
    bool valueRestrictionsConjunctionsElementNegated{true};
    valueRestrictionsConjunctionsElement.negated = valueRestrictionsConjunctionsElementNegated;
    ::ad::map::restriction::RoadUserTypeList valueRestrictionsConjunctionsElementRoadUserTypes;
    ::ad::map::restriction::RoadUserType valueRestrictionsConjunctionsElementRoadUserTypesElement(
      ::ad::map::restriction::RoadUserType::INVALID);
    valueRestrictionsConjunctionsElementRoadUserTypes.resize(1,
                                                             valueRestrictionsConjunctionsElementRoadUserTypesElement);
    valueRestrictionsConjunctionsElement.roadUserTypes = valueRestrictionsConjunctionsElementRoadUserTypes;
    ::ad::map::restriction::PassengerCount valueRestrictionsConjunctionsElementPassengersMin(
      std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
    valueRestrictionsConjunctionsElement.passengersMin = valueRestrictionsConjunctionsElementPassengersMin;
    valueRestrictionsConjunctions.resize(1, valueRestrictionsConjunctionsElement);
    valueRestrictions.conjunctions = valueRestrictionsConjunctions;
    ::ad::map::restriction::RestrictionList valueRestrictionsDisjunctions;
    ::ad::map::restriction::Restriction valueRestrictionsDisjunctionsElement;
    bool valueRestrictionsDisjunctionsElementNegated{true};
    valueRestrictionsDisjunctionsElement.negated = valueRestrictionsDisjunctionsElementNegated;
    ::ad::map::restriction::RoadUserTypeList valueRestrictionsDisjunctionsElementRoadUserTypes;
    ::ad::map::restriction::RoadUserType valueRestrictionsDisjunctionsElementRoadUserTypesElement(
      ::ad::map::restriction::RoadUserType::INVALID);
    valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1,
                                                             valueRestrictionsDisjunctionsElementRoadUserTypesElement);
    valueRestrictionsDisjunctionsElement.roadUserTypes = valueRestrictionsDisjunctionsElementRoadUserTypes;
    ::ad::map::restriction::PassengerCount valueRestrictionsDisjunctionsElementPassengersMin(
      std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
    valueRestrictionsDisjunctionsElement.passengersMin = valueRestrictionsDisjunctionsElementPassengersMin;
    valueRestrictionsDisjunctions.resize(1, valueRestrictionsDisjunctionsElement);
    valueRestrictions.disjunctions = valueRestrictionsDisjunctions;
    value.restrictions = valueRestrictions;
    ::ad::physics::Distance valueLength(-1e9);
    value.length = valueLength;
    ::ad::physics::MetricRange valueLengthRange;
    ::ad::physics::Distance valueLengthRangeMinimum(-1e9);
    valueLengthRangeMinimum = ::ad::physics::Distance(0.); // set to valid value within struct
    valueLengthRange.minimum = valueLengthRangeMinimum;
    ::ad::physics::Distance valueLengthRangeMaximum(-1e9);
    valueLengthRange.maximum = valueLengthRangeMaximum;
    valueLengthRange.maximum = valueLengthRange.minimum;
    valueLengthRange.minimum = valueLengthRange.maximum;
    value.lengthRange = valueLengthRange;
    ::ad::physics::Distance valueWidth(-1e9);
    value.width = valueWidth;
    ::ad::physics::MetricRange valueWidthRange;
    ::ad::physics::Distance valueWidthRangeMinimum(-1e9);
    valueWidthRangeMinimum = ::ad::physics::Distance(0.); // set to valid value within struct
    valueWidthRange.minimum = valueWidthRangeMinimum;
    ::ad::physics::Distance valueWidthRangeMaximum(-1e9);
    valueWidthRange.maximum = valueWidthRangeMaximum;
    valueWidthRange.maximum = valueWidthRange.minimum;
    valueWidthRange.minimum = valueWidthRange.maximum;
    value.widthRange = valueWidthRange;
    ::ad::map::restriction::SpeedLimitList valueSpeedLimits;
    ::ad::map::restriction::SpeedLimit valueSpeedLimitsElement;
    ::ad::physics::Speed valueSpeedLimitsElementSpeedLimit(-100.);
    valueSpeedLimitsElement.speedLimit = valueSpeedLimitsElementSpeedLimit;
    ::ad::physics::ParametricRange valueSpeedLimitsElementLanePiece;
    ::ad::physics::ParametricValue valueSpeedLimitsElementLanePieceMinimum(0.);
    valueSpeedLimitsElementLanePiece.minimum = valueSpeedLimitsElementLanePieceMinimum;
    ::ad::physics::ParametricValue valueSpeedLimitsElementLanePieceMaximum(0.);
    valueSpeedLimitsElementLanePiece.maximum = valueSpeedLimitsElementLanePieceMaximum;
    valueSpeedLimitsElementLanePiece.maximum = valueSpeedLimitsElementLanePiece.minimum;
    valueSpeedLimitsElementLanePiece.minimum = valueSpeedLimitsElementLanePiece.maximum;
    valueSpeedLimitsElement.lanePiece = valueSpeedLimitsElementLanePiece;
    valueSpeedLimits.resize(1, valueSpeedLimitsElement);
    value.speedLimits = valueSpeedLimits;
    ::ad::map::point::Geometry valueEdgeLeft;
    bool valueEdgeLeftIsValid{true};
    valueEdgeLeft.isValid = valueEdgeLeftIsValid;
    bool valueEdgeLeftIsClosed{true};
    valueEdgeLeft.isClosed = valueEdgeLeftIsClosed;
    ::ad::map::point::ECEFEdge valueEdgeLeftEcefEdge;
    ::ad::map::point::ECEFPoint valueEdgeLeftEcefEdgeElement;
    ::ad::map::point::ECEFCoordinate valueEdgeLeftEcefEdgeElementX(-6400000);
    valueEdgeLeftEcefEdgeElement.x = valueEdgeLeftEcefEdgeElementX;
    ::ad::map::point::ECEFCoordinate valueEdgeLeftEcefEdgeElementY(-6400000);
    valueEdgeLeftEcefEdgeElement.y = valueEdgeLeftEcefEdgeElementY;
    ::ad::map::point::ECEFCoordinate valueEdgeLeftEcefEdgeElementZ(-6400000);
    valueEdgeLeftEcefEdgeElement.z = valueEdgeLeftEcefEdgeElementZ;
    valueEdgeLeftEcefEdge.resize(1, valueEdgeLeftEcefEdgeElement);
    valueEdgeLeft.ecefEdge = valueEdgeLeftEcefEdge;
    ::ad::physics::Distance valueEdgeLeftLength(-1e9);
    valueEdgeLeft.length = valueEdgeLeftLength;
    ::ad::map::point::ENUEdgeCache valueEdgeLeftPrivate_enuEdgeCache;
    ::ad::map::point::ENUEdge valueEdgeLeftPrivate_enuEdgeCacheEnuEdge;
    ::ad::map::point::ENUPoint valueEdgeLeftPrivate_enuEdgeCacheEnuEdgeElement;
    ::ad::map::point::ENUCoordinate valueEdgeLeftPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
    valueEdgeLeftPrivate_enuEdgeCacheEnuEdgeElement.x = valueEdgeLeftPrivate_enuEdgeCacheEnuEdgeElementX;
    ::ad::map::point::ENUCoordinate valueEdgeLeftPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
    valueEdgeLeftPrivate_enuEdgeCacheEnuEdgeElement.y = valueEdgeLeftPrivate_enuEdgeCacheEnuEdgeElementY;
    ::ad::map::point::ENUCoordinate valueEdgeLeftPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
    valueEdgeLeftPrivate_enuEdgeCacheEnuEdgeElement.z = valueEdgeLeftPrivate_enuEdgeCacheEnuEdgeElementZ;
    valueEdgeLeftPrivate_enuEdgeCacheEnuEdge.resize(1, valueEdgeLeftPrivate_enuEdgeCacheEnuEdgeElement);
    valueEdgeLeftPrivate_enuEdgeCache.enuEdge = valueEdgeLeftPrivate_enuEdgeCacheEnuEdge;
    uint64_t valueEdgeLeftPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
    valueEdgeLeftPrivate_enuEdgeCache.enuVersion = valueEdgeLeftPrivate_enuEdgeCacheEnuVersion;
    valueEdgeLeft.private_enuEdgeCache = valueEdgeLeftPrivate_enuEdgeCache;
    value.edgeLeft = valueEdgeLeft;
    ::ad::map::point::Geometry valueEdgeRight;
    bool valueEdgeRightIsValid{true};
    valueEdgeRight.isValid = valueEdgeRightIsValid;
    bool valueEdgeRightIsClosed{true};
    valueEdgeRight.isClosed = valueEdgeRightIsClosed;
    ::ad::map::point::ECEFEdge valueEdgeRightEcefEdge;
    ::ad::map::point::ECEFPoint valueEdgeRightEcefEdgeElement;
    ::ad::map::point::ECEFCoordinate valueEdgeRightEcefEdgeElementX(-6400000);
    valueEdgeRightEcefEdgeElement.x = valueEdgeRightEcefEdgeElementX;
    ::ad::map::point::ECEFCoordinate valueEdgeRightEcefEdgeElementY(-6400000);
    valueEdgeRightEcefEdgeElement.y = valueEdgeRightEcefEdgeElementY;
    ::ad::map::point::ECEFCoordinate valueEdgeRightEcefEdgeElementZ(-6400000);
    valueEdgeRightEcefEdgeElement.z = valueEdgeRightEcefEdgeElementZ;
    valueEdgeRightEcefEdge.resize(1, valueEdgeRightEcefEdgeElement);
    valueEdgeRight.ecefEdge = valueEdgeRightEcefEdge;
    ::ad::physics::Distance valueEdgeRightLength(-1e9);
    valueEdgeRight.length = valueEdgeRightLength;
    ::ad::map::point::ENUEdgeCache valueEdgeRightPrivate_enuEdgeCache;
    ::ad::map::point::ENUEdge valueEdgeRightPrivate_enuEdgeCacheEnuEdge;
    ::ad::map::point::ENUPoint valueEdgeRightPrivate_enuEdgeCacheEnuEdgeElement;
    ::ad::map::point::ENUCoordinate valueEdgeRightPrivate_enuEdgeCacheEnuEdgeElementX(-16384);
    valueEdgeRightPrivate_enuEdgeCacheEnuEdgeElement.x = valueEdgeRightPrivate_enuEdgeCacheEnuEdgeElementX;
    ::ad::map::point::ENUCoordinate valueEdgeRightPrivate_enuEdgeCacheEnuEdgeElementY(-16384);
    valueEdgeRightPrivate_enuEdgeCacheEnuEdgeElement.y = valueEdgeRightPrivate_enuEdgeCacheEnuEdgeElementY;
    ::ad::map::point::ENUCoordinate valueEdgeRightPrivate_enuEdgeCacheEnuEdgeElementZ(-16384);
    valueEdgeRightPrivate_enuEdgeCacheEnuEdgeElement.z = valueEdgeRightPrivate_enuEdgeCacheEnuEdgeElementZ;
    valueEdgeRightPrivate_enuEdgeCacheEnuEdge.resize(1, valueEdgeRightPrivate_enuEdgeCacheEnuEdgeElement);
    valueEdgeRightPrivate_enuEdgeCache.enuEdge = valueEdgeRightPrivate_enuEdgeCacheEnuEdge;
    uint64_t valueEdgeRightPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::min()};
    valueEdgeRightPrivate_enuEdgeCache.enuVersion = valueEdgeRightPrivate_enuEdgeCacheEnuVersion;
    valueEdgeRight.private_enuEdgeCache = valueEdgeRightPrivate_enuEdgeCache;
    value.edgeRight = valueEdgeRight;
    ::ad::map::lane::ContactLaneList valueContactLanes;
    ::ad::map::lane::ContactLane valueContactLanesElement;
    ::ad::map::lane::LaneId valueContactLanesElementToLane(1);
    valueContactLanesElement.toLane = valueContactLanesElementToLane;
    ::ad::map::lane::ContactLocation valueContactLanesElementLocation(::ad::map::lane::ContactLocation::INVALID);
    valueContactLanesElement.location = valueContactLanesElementLocation;
    ::ad::map::lane::ContactTypeList valueContactLanesElementTypes;
    ::ad::map::lane::ContactType valueContactLanesElementTypesElement(::ad::map::lane::ContactType::INVALID);
    valueContactLanesElementTypes.resize(1, valueContactLanesElementTypesElement);
    valueContactLanesElement.types = valueContactLanesElementTypes;
    ::ad::map::restriction::Restrictions valueContactLanesElementRestrictions;
    ::ad::map::restriction::RestrictionList valueContactLanesElementRestrictionsConjunctions;
    ::ad::map::restriction::Restriction valueContactLanesElementRestrictionsConjunctionsElement;
    bool valueContactLanesElementRestrictionsConjunctionsElementNegated{true};
    valueContactLanesElementRestrictionsConjunctionsElement.negated
      = valueContactLanesElementRestrictionsConjunctionsElementNegated;
    ::ad::map::restriction::RoadUserTypeList valueContactLanesElementRestrictionsConjunctionsElementRoadUserTypes;
    ::ad::map::restriction::RoadUserType valueContactLanesElementRestrictionsConjunctionsElementRoadUserTypesElement(
      ::ad::map::restriction::RoadUserType::INVALID);
    valueContactLanesElementRestrictionsConjunctionsElementRoadUserTypes.resize(
      1, valueContactLanesElementRestrictionsConjunctionsElementRoadUserTypesElement);
    valueContactLanesElementRestrictionsConjunctionsElement.roadUserTypes
      = valueContactLanesElementRestrictionsConjunctionsElementRoadUserTypes;
    ::ad::map::restriction::PassengerCount valueContactLanesElementRestrictionsConjunctionsElementPassengersMin(
      std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
    valueContactLanesElementRestrictionsConjunctionsElement.passengersMin
      = valueContactLanesElementRestrictionsConjunctionsElementPassengersMin;
    valueContactLanesElementRestrictionsConjunctions.resize(1, valueContactLanesElementRestrictionsConjunctionsElement);
    valueContactLanesElementRestrictions.conjunctions = valueContactLanesElementRestrictionsConjunctions;
    ::ad::map::restriction::RestrictionList valueContactLanesElementRestrictionsDisjunctions;
    ::ad::map::restriction::Restriction valueContactLanesElementRestrictionsDisjunctionsElement;
    bool valueContactLanesElementRestrictionsDisjunctionsElementNegated{true};
    valueContactLanesElementRestrictionsDisjunctionsElement.negated
      = valueContactLanesElementRestrictionsDisjunctionsElementNegated;
    ::ad::map::restriction::RoadUserTypeList valueContactLanesElementRestrictionsDisjunctionsElementRoadUserTypes;
    ::ad::map::restriction::RoadUserType valueContactLanesElementRestrictionsDisjunctionsElementRoadUserTypesElement(
      ::ad::map::restriction::RoadUserType::INVALID);
    valueContactLanesElementRestrictionsDisjunctionsElementRoadUserTypes.resize(
      1, valueContactLanesElementRestrictionsDisjunctionsElementRoadUserTypesElement);
    valueContactLanesElementRestrictionsDisjunctionsElement.roadUserTypes
      = valueContactLanesElementRestrictionsDisjunctionsElementRoadUserTypes;
    ::ad::map::restriction::PassengerCount valueContactLanesElementRestrictionsDisjunctionsElementPassengersMin(
      std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
    valueContactLanesElementRestrictionsDisjunctionsElement.passengersMin
      = valueContactLanesElementRestrictionsDisjunctionsElementPassengersMin;
    valueContactLanesElementRestrictionsDisjunctions.resize(1, valueContactLanesElementRestrictionsDisjunctionsElement);
    valueContactLanesElementRestrictions.disjunctions = valueContactLanesElementRestrictionsDisjunctions;
    valueContactLanesElement.restrictions = valueContactLanesElementRestrictions;
    ::ad::map::landmark::LandmarkId valueContactLanesElementTrafficLightId(
      std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
    valueContactLanesElement.trafficLightId = valueContactLanesElementTrafficLightId;
    valueContactLanes.resize(1, valueContactLanesElement);
    value.contactLanes = valueContactLanes;
    ::ad::map::lane::ComplianceVersion valueComplianceVersion(
      std::numeric_limits<::ad::map::lane::ComplianceVersion>::lowest());
    value.complianceVersion = valueComplianceVersion;
    ::ad::map::point::BoundingSphere valueBoundingSphere;
    ::ad::map::point::ECEFPoint valueBoundingSphereCenter;
    ::ad::map::point::ECEFCoordinate valueBoundingSphereCenterX(-6400000);
    valueBoundingSphereCenter.x = valueBoundingSphereCenterX;
    ::ad::map::point::ECEFCoordinate valueBoundingSphereCenterY(-6400000);
    valueBoundingSphereCenter.y = valueBoundingSphereCenterY;
    ::ad::map::point::ECEFCoordinate valueBoundingSphereCenterZ(-6400000);
    valueBoundingSphereCenter.z = valueBoundingSphereCenterZ;
    valueBoundingSphere.center = valueBoundingSphereCenter;
    ::ad::physics::Distance valueBoundingSphereRadius(-1e9);
    valueBoundingSphere.radius = valueBoundingSphereRadius;
    value.boundingSphere = valueBoundingSphere;
    ::ad::map::landmark::LandmarkIdList valueVisibleLandmarks;
    ::ad::map::landmark::LandmarkId valueVisibleLandmarksElement(
      std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
    valueVisibleLandmarks.resize(1, valueVisibleLandmarksElement);
    value.visibleLandmarks = valueVisibleLandmarks;
    mValue = value;
  }

  ::ad::map::lane::Lane mValue;
};

TEST_F(LaneTests, copyConstruction)
{
  ::ad::map::lane::Lane value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneTests, moveConstruction)
{
  ::ad::map::lane::Lane tmpValue(mValue);
  ::ad::map::lane::Lane value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneTests, copyAssignment)
{
  ::ad::map::lane::Lane value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneTests, moveAssignment)
{
  ::ad::map::lane::Lane tmpValue(mValue);
  ::ad::map::lane::Lane value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneTests, comparisonOperatorEqual)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(LaneTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(LaneTests, comparisonOperatorIdDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::map::lane::LaneId id(std::numeric_limits<::ad::map::lane::LaneId>::max());
  valueA.id = id;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorTypeDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::map::lane::LaneType type(::ad::map::lane::LaneType::BIKE);
  valueA.type = type;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorDirectionDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::map::lane::LaneDirection direction(::ad::map::lane::LaneDirection::NONE);
  valueA.direction = direction;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorRestrictionsDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::map::restriction::Restrictions restrictions;
  ::ad::map::restriction::RestrictionList restrictionsConjunctions;
  ::ad::map::restriction::Restriction restrictionsConjunctionsElement;
  bool restrictionsConjunctionsElementNegated{false};
  restrictionsConjunctionsElement.negated = restrictionsConjunctionsElementNegated;
  ::ad::map::restriction::RoadUserTypeList restrictionsConjunctionsElementRoadUserTypes;
  ::ad::map::restriction::RoadUserType restrictionsConjunctionsElementRoadUserTypesElement(
    ::ad::map::restriction::RoadUserType::CAR_DIESEL);
  restrictionsConjunctionsElementRoadUserTypes.resize(2, restrictionsConjunctionsElementRoadUserTypesElement);
  restrictionsConjunctionsElement.roadUserTypes = restrictionsConjunctionsElementRoadUserTypes;
  ::ad::map::restriction::PassengerCount restrictionsConjunctionsElementPassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::max());
  restrictionsConjunctionsElement.passengersMin = restrictionsConjunctionsElementPassengersMin;
  restrictionsConjunctions.resize(2, restrictionsConjunctionsElement);
  restrictions.conjunctions = restrictionsConjunctions;
  ::ad::map::restriction::RestrictionList restrictionsDisjunctions;
  ::ad::map::restriction::Restriction restrictionsDisjunctionsElement;
  bool restrictionsDisjunctionsElementNegated{false};
  restrictionsDisjunctionsElement.negated = restrictionsDisjunctionsElementNegated;
  ::ad::map::restriction::RoadUserTypeList restrictionsDisjunctionsElementRoadUserTypes;
  ::ad::map::restriction::RoadUserType restrictionsDisjunctionsElementRoadUserTypesElement(
    ::ad::map::restriction::RoadUserType::CAR_DIESEL);
  restrictionsDisjunctionsElementRoadUserTypes.resize(2, restrictionsDisjunctionsElementRoadUserTypesElement);
  restrictionsDisjunctionsElement.roadUserTypes = restrictionsDisjunctionsElementRoadUserTypes;
  ::ad::map::restriction::PassengerCount restrictionsDisjunctionsElementPassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::max());
  restrictionsDisjunctionsElement.passengersMin = restrictionsDisjunctionsElementPassengersMin;
  restrictionsDisjunctions.resize(2, restrictionsDisjunctionsElement);
  restrictions.disjunctions = restrictionsDisjunctions;
  valueA.restrictions = restrictions;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorLengthDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::physics::Distance length(1e9);
  valueA.length = length;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorLengthRangeDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::physics::MetricRange lengthRange;
  ::ad::physics::Distance lengthRangeMinimum(1e9);
  lengthRange.minimum = lengthRangeMinimum;
  ::ad::physics::Distance lengthRangeMaximum(1e9);
  lengthRangeMaximum = ::ad::physics::Distance(1e6); // set to valid value within struct
  lengthRange.maximum = lengthRangeMaximum;
  lengthRange.maximum = lengthRange.minimum;
  lengthRange.minimum = lengthRange.maximum;
  valueA.lengthRange = lengthRange;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorWidthDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::physics::Distance width(1e9);
  valueA.width = width;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorWidthRangeDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::physics::MetricRange widthRange;
  ::ad::physics::Distance widthRangeMinimum(1e9);
  widthRange.minimum = widthRangeMinimum;
  ::ad::physics::Distance widthRangeMaximum(1e9);
  widthRangeMaximum = ::ad::physics::Distance(1e6); // set to valid value within struct
  widthRange.maximum = widthRangeMaximum;
  widthRange.maximum = widthRange.minimum;
  widthRange.minimum = widthRange.maximum;
  valueA.widthRange = widthRange;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorSpeedLimitsDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::map::restriction::SpeedLimitList speedLimits;
  ::ad::map::restriction::SpeedLimit speedLimitsElement;
  ::ad::physics::Speed speedLimitsElementSpeedLimit(100.);
  speedLimitsElement.speedLimit = speedLimitsElementSpeedLimit;
  ::ad::physics::ParametricRange speedLimitsElementLanePiece;
  ::ad::physics::ParametricValue speedLimitsElementLanePieceMinimum(1.);
  speedLimitsElementLanePiece.minimum = speedLimitsElementLanePieceMinimum;
  ::ad::physics::ParametricValue speedLimitsElementLanePieceMaximum(1.);
  speedLimitsElementLanePiece.maximum = speedLimitsElementLanePieceMaximum;
  speedLimitsElementLanePiece.maximum = speedLimitsElementLanePiece.minimum;
  speedLimitsElementLanePiece.minimum = speedLimitsElementLanePiece.maximum;
  speedLimitsElement.lanePiece = speedLimitsElementLanePiece;
  speedLimits.resize(2, speedLimitsElement);
  valueA.speedLimits = speedLimits;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorEdgeLeftDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::map::point::Geometry edgeLeft;
  bool edgeLeftIsValid{false};
  edgeLeft.isValid = edgeLeftIsValid;
  bool edgeLeftIsClosed{false};
  edgeLeft.isClosed = edgeLeftIsClosed;
  ::ad::map::point::ECEFEdge edgeLeftEcefEdge;
  ::ad::map::point::ECEFPoint edgeLeftEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate edgeLeftEcefEdgeElementX(6400000);
  edgeLeftEcefEdgeElement.x = edgeLeftEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate edgeLeftEcefEdgeElementY(6400000);
  edgeLeftEcefEdgeElement.y = edgeLeftEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate edgeLeftEcefEdgeElementZ(6400000);
  edgeLeftEcefEdgeElement.z = edgeLeftEcefEdgeElementZ;
  edgeLeftEcefEdge.resize(2, edgeLeftEcefEdgeElement);
  edgeLeft.ecefEdge = edgeLeftEcefEdge;
  ::ad::physics::Distance edgeLeftLength(1e9);
  edgeLeft.length = edgeLeftLength;
  ::ad::map::point::ENUEdgeCache edgeLeftPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge edgeLeftPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint edgeLeftPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate edgeLeftPrivate_enuEdgeCacheEnuEdgeElementX(16384);
  edgeLeftPrivate_enuEdgeCacheEnuEdgeElement.x = edgeLeftPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate edgeLeftPrivate_enuEdgeCacheEnuEdgeElementY(16384);
  edgeLeftPrivate_enuEdgeCacheEnuEdgeElement.y = edgeLeftPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate edgeLeftPrivate_enuEdgeCacheEnuEdgeElementZ(16384);
  edgeLeftPrivate_enuEdgeCacheEnuEdgeElement.z = edgeLeftPrivate_enuEdgeCacheEnuEdgeElementZ;
  edgeLeftPrivate_enuEdgeCacheEnuEdge.resize(2, edgeLeftPrivate_enuEdgeCacheEnuEdgeElement);
  edgeLeftPrivate_enuEdgeCache.enuEdge = edgeLeftPrivate_enuEdgeCacheEnuEdge;
  uint64_t edgeLeftPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::max()};
  edgeLeftPrivate_enuEdgeCache.enuVersion = edgeLeftPrivate_enuEdgeCacheEnuVersion;
  edgeLeft.private_enuEdgeCache = edgeLeftPrivate_enuEdgeCache;
  valueA.edgeLeft = edgeLeft;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorEdgeRightDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::map::point::Geometry edgeRight;
  bool edgeRightIsValid{false};
  edgeRight.isValid = edgeRightIsValid;
  bool edgeRightIsClosed{false};
  edgeRight.isClosed = edgeRightIsClosed;
  ::ad::map::point::ECEFEdge edgeRightEcefEdge;
  ::ad::map::point::ECEFPoint edgeRightEcefEdgeElement;
  ::ad::map::point::ECEFCoordinate edgeRightEcefEdgeElementX(6400000);
  edgeRightEcefEdgeElement.x = edgeRightEcefEdgeElementX;
  ::ad::map::point::ECEFCoordinate edgeRightEcefEdgeElementY(6400000);
  edgeRightEcefEdgeElement.y = edgeRightEcefEdgeElementY;
  ::ad::map::point::ECEFCoordinate edgeRightEcefEdgeElementZ(6400000);
  edgeRightEcefEdgeElement.z = edgeRightEcefEdgeElementZ;
  edgeRightEcefEdge.resize(2, edgeRightEcefEdgeElement);
  edgeRight.ecefEdge = edgeRightEcefEdge;
  ::ad::physics::Distance edgeRightLength(1e9);
  edgeRight.length = edgeRightLength;
  ::ad::map::point::ENUEdgeCache edgeRightPrivate_enuEdgeCache;
  ::ad::map::point::ENUEdge edgeRightPrivate_enuEdgeCacheEnuEdge;
  ::ad::map::point::ENUPoint edgeRightPrivate_enuEdgeCacheEnuEdgeElement;
  ::ad::map::point::ENUCoordinate edgeRightPrivate_enuEdgeCacheEnuEdgeElementX(16384);
  edgeRightPrivate_enuEdgeCacheEnuEdgeElement.x = edgeRightPrivate_enuEdgeCacheEnuEdgeElementX;
  ::ad::map::point::ENUCoordinate edgeRightPrivate_enuEdgeCacheEnuEdgeElementY(16384);
  edgeRightPrivate_enuEdgeCacheEnuEdgeElement.y = edgeRightPrivate_enuEdgeCacheEnuEdgeElementY;
  ::ad::map::point::ENUCoordinate edgeRightPrivate_enuEdgeCacheEnuEdgeElementZ(16384);
  edgeRightPrivate_enuEdgeCacheEnuEdgeElement.z = edgeRightPrivate_enuEdgeCacheEnuEdgeElementZ;
  edgeRightPrivate_enuEdgeCacheEnuEdge.resize(2, edgeRightPrivate_enuEdgeCacheEnuEdgeElement);
  edgeRightPrivate_enuEdgeCache.enuEdge = edgeRightPrivate_enuEdgeCacheEnuEdge;
  uint64_t edgeRightPrivate_enuEdgeCacheEnuVersion{std::numeric_limits<uint64_t>::max()};
  edgeRightPrivate_enuEdgeCache.enuVersion = edgeRightPrivate_enuEdgeCacheEnuVersion;
  edgeRight.private_enuEdgeCache = edgeRightPrivate_enuEdgeCache;
  valueA.edgeRight = edgeRight;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorContactLanesDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::map::lane::ContactLaneList contactLanes;
  ::ad::map::lane::ContactLane contactLanesElement;
  ::ad::map::lane::LaneId contactLanesElementToLane(std::numeric_limits<::ad::map::lane::LaneId>::max());
  contactLanesElement.toLane = contactLanesElementToLane;
  ::ad::map::lane::ContactLocation contactLanesElementLocation(::ad::map::lane::ContactLocation::OVERLAP);
  contactLanesElement.location = contactLanesElementLocation;
  ::ad::map::lane::ContactTypeList contactLanesElementTypes;
  ::ad::map::lane::ContactType contactLanesElementTypesElement(
    ::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT);
  contactLanesElementTypes.resize(2, contactLanesElementTypesElement);
  contactLanesElement.types = contactLanesElementTypes;
  ::ad::map::restriction::Restrictions contactLanesElementRestrictions;
  ::ad::map::restriction::RestrictionList contactLanesElementRestrictionsConjunctions;
  ::ad::map::restriction::Restriction contactLanesElementRestrictionsConjunctionsElement;
  bool contactLanesElementRestrictionsConjunctionsElementNegated{false};
  contactLanesElementRestrictionsConjunctionsElement.negated
    = contactLanesElementRestrictionsConjunctionsElementNegated;
  ::ad::map::restriction::RoadUserTypeList contactLanesElementRestrictionsConjunctionsElementRoadUserTypes;
  ::ad::map::restriction::RoadUserType contactLanesElementRestrictionsConjunctionsElementRoadUserTypesElement(
    ::ad::map::restriction::RoadUserType::CAR_DIESEL);
  contactLanesElementRestrictionsConjunctionsElementRoadUserTypes.resize(
    2, contactLanesElementRestrictionsConjunctionsElementRoadUserTypesElement);
  contactLanesElementRestrictionsConjunctionsElement.roadUserTypes
    = contactLanesElementRestrictionsConjunctionsElementRoadUserTypes;
  ::ad::map::restriction::PassengerCount contactLanesElementRestrictionsConjunctionsElementPassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::max());
  contactLanesElementRestrictionsConjunctionsElement.passengersMin
    = contactLanesElementRestrictionsConjunctionsElementPassengersMin;
  contactLanesElementRestrictionsConjunctions.resize(2, contactLanesElementRestrictionsConjunctionsElement);
  contactLanesElementRestrictions.conjunctions = contactLanesElementRestrictionsConjunctions;
  ::ad::map::restriction::RestrictionList contactLanesElementRestrictionsDisjunctions;
  ::ad::map::restriction::Restriction contactLanesElementRestrictionsDisjunctionsElement;
  bool contactLanesElementRestrictionsDisjunctionsElementNegated{false};
  contactLanesElementRestrictionsDisjunctionsElement.negated
    = contactLanesElementRestrictionsDisjunctionsElementNegated;
  ::ad::map::restriction::RoadUserTypeList contactLanesElementRestrictionsDisjunctionsElementRoadUserTypes;
  ::ad::map::restriction::RoadUserType contactLanesElementRestrictionsDisjunctionsElementRoadUserTypesElement(
    ::ad::map::restriction::RoadUserType::CAR_DIESEL);
  contactLanesElementRestrictionsDisjunctionsElementRoadUserTypes.resize(
    2, contactLanesElementRestrictionsDisjunctionsElementRoadUserTypesElement);
  contactLanesElementRestrictionsDisjunctionsElement.roadUserTypes
    = contactLanesElementRestrictionsDisjunctionsElementRoadUserTypes;
  ::ad::map::restriction::PassengerCount contactLanesElementRestrictionsDisjunctionsElementPassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::max());
  contactLanesElementRestrictionsDisjunctionsElement.passengersMin
    = contactLanesElementRestrictionsDisjunctionsElementPassengersMin;
  contactLanesElementRestrictionsDisjunctions.resize(2, contactLanesElementRestrictionsDisjunctionsElement);
  contactLanesElementRestrictions.disjunctions = contactLanesElementRestrictionsDisjunctions;
  contactLanesElement.restrictions = contactLanesElementRestrictions;
  ::ad::map::landmark::LandmarkId contactLanesElementTrafficLightId(
    std::numeric_limits<::ad::map::landmark::LandmarkId>::max());
  contactLanesElement.trafficLightId = contactLanesElementTrafficLightId;
  contactLanes.resize(2, contactLanesElement);
  valueA.contactLanes = contactLanes;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorComplianceVersionDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::map::lane::ComplianceVersion complianceVersion(std::numeric_limits<::ad::map::lane::ComplianceVersion>::max());
  valueA.complianceVersion = complianceVersion;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorBoundingSphereDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::map::point::BoundingSphere boundingSphere;
  ::ad::map::point::ECEFPoint boundingSphereCenter;
  ::ad::map::point::ECEFCoordinate boundingSphereCenterX(6400000);
  boundingSphereCenter.x = boundingSphereCenterX;
  ::ad::map::point::ECEFCoordinate boundingSphereCenterY(6400000);
  boundingSphereCenter.y = boundingSphereCenterY;
  ::ad::map::point::ECEFCoordinate boundingSphereCenterZ(6400000);
  boundingSphereCenter.z = boundingSphereCenterZ;
  boundingSphere.center = boundingSphereCenter;
  ::ad::physics::Distance boundingSphereRadius(1e9);
  boundingSphere.radius = boundingSphereRadius;
  valueA.boundingSphere = boundingSphere;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneTests, comparisonOperatorVisibleLandmarksDiffers)
{
  ::ad::map::lane::Lane valueA = mValue;
  ::ad::map::landmark::LandmarkIdList visibleLandmarks;
  ::ad::map::landmark::LandmarkId visibleLandmarksElement(std::numeric_limits<::ad::map::landmark::LandmarkId>::max());
  visibleLandmarks.resize(2, visibleLandmarksElement);
  valueA.visibleLandmarks = visibleLandmarks;
  ::ad::map::lane::Lane valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
