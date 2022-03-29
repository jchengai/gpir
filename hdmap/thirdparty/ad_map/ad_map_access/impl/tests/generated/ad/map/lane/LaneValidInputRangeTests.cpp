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

#include "ad/map/lane/LaneValidInputRange.hpp"

TEST(LaneValidInputRangeTests, testValidInputRange)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeTypeTooSmall)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value below input range minimum
  ::ad::map::lane::LaneType invalidInitializedMember(static_cast<::ad::map::lane::LaneType>(-1));
  value.type = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeTypeTooBig)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value above input range maximum
  ::ad::map::lane::LaneType invalidInitializedMember(static_cast<::ad::map::lane::LaneType>(-1));
  value.type = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeDirectionTooSmall)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value below input range minimum
  ::ad::map::lane::LaneDirection invalidInitializedMember(static_cast<::ad::map::lane::LaneDirection>(-1));
  value.direction = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeDirectionTooBig)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value above input range maximum
  ::ad::map::lane::LaneDirection invalidInitializedMember(static_cast<::ad::map::lane::LaneDirection>(-1));
  value.direction = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeLengthTooSmall)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.length = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeLengthTooBig)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.length = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangelengthDefault)
{
  ::ad::map::lane::Lane value;
  ::ad::physics::Distance valueDefault;
  value.length = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeLengthRangeTooSmall)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value below input range minimum
  ::ad::physics::MetricRange invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberMinimum(-1e9 * 1.1);
  invalidInitializedMember.minimum = invalidInitializedMemberMinimum;
  value.lengthRange = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeLengthRangeTooBig)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value above input range maximum
  ::ad::physics::MetricRange invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberMinimum(1e9 * 1.1);
  invalidInitializedMember.minimum = invalidInitializedMemberMinimum;
  value.lengthRange = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeWidthTooSmall)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value below input range minimum
  ::ad::physics::Distance invalidInitializedMember(-1e9 * 1.1);
  value.width = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeWidthTooBig)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value above input range maximum
  ::ad::physics::Distance invalidInitializedMember(1e9 * 1.1);
  value.width = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangewidthDefault)
{
  ::ad::map::lane::Lane value;
  ::ad::physics::Distance valueDefault;
  value.width = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeWidthRangeTooSmall)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value below input range minimum
  ::ad::physics::MetricRange invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberMinimum(-1e9 * 1.1);
  invalidInitializedMember.minimum = invalidInitializedMemberMinimum;
  value.widthRange = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeWidthRangeTooBig)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value above input range maximum
  ::ad::physics::MetricRange invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberMinimum(1e9 * 1.1);
  invalidInitializedMember.minimum = invalidInitializedMemberMinimum;
  value.widthRange = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeEdgeLeftTooSmall)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value below input range minimum
  ::ad::map::point::Geometry invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberLength(-1e9 * 1.1);
  invalidInitializedMember.length = invalidInitializedMemberLength;
  value.edgeLeft = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeEdgeLeftTooBig)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value above input range maximum
  ::ad::map::point::Geometry invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberLength(1e9 * 1.1);
  invalidInitializedMember.length = invalidInitializedMemberLength;
  value.edgeLeft = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeEdgeRightTooSmall)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value below input range minimum
  ::ad::map::point::Geometry invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberLength(-1e9 * 1.1);
  invalidInitializedMember.length = invalidInitializedMemberLength;
  value.edgeRight = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeEdgeRightTooBig)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value above input range maximum
  ::ad::map::point::Geometry invalidInitializedMember;
  ::ad::physics::Distance invalidInitializedMemberLength(1e9 * 1.1);
  invalidInitializedMember.length = invalidInitializedMemberLength;
  value.edgeRight = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeBoundingSphereTooSmall)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value below input range minimum
  ::ad::map::point::BoundingSphere invalidInitializedMember;
  ::ad::map::point::ECEFPoint invalidInitializedMemberCenter;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberCenterX(-6400000 * 1.1);
  invalidInitializedMemberCenter.x = invalidInitializedMemberCenterX;
  invalidInitializedMember.center = invalidInitializedMemberCenter;
  value.boundingSphere = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneValidInputRangeTests, testValidInputRangeBoundingSphereTooBig)
{
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
  valueRestrictionsConjunctionsElementRoadUserTypes.resize(1, valueRestrictionsConjunctionsElementRoadUserTypesElement);
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
  valueRestrictionsDisjunctionsElementRoadUserTypes.resize(1, valueRestrictionsDisjunctionsElementRoadUserTypesElement);
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

  // override member with data type value above input range maximum
  ::ad::map::point::BoundingSphere invalidInitializedMember;
  ::ad::map::point::ECEFPoint invalidInitializedMemberCenter;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberCenterX(6400000 * 1.1);
  invalidInitializedMemberCenter.x = invalidInitializedMemberCenterX;
  invalidInitializedMember.center = invalidInitializedMemberCenter;
  value.boundingSphere = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
