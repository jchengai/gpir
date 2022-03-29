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

#include "ad/map/lane/ContactLaneValidInputRange.hpp"

TEST(ContactLaneValidInputRangeTests, testValidInputRange)
{
  ::ad::map::lane::ContactLane value;
  ::ad::map::lane::LaneId valueToLane(1);
  value.toLane = valueToLane;
  ::ad::map::lane::ContactLocation valueLocation(::ad::map::lane::ContactLocation::INVALID);
  value.location = valueLocation;
  ::ad::map::lane::ContactTypeList valueTypes;
  ::ad::map::lane::ContactType valueTypesElement(::ad::map::lane::ContactType::INVALID);
  valueTypes.resize(1, valueTypesElement);
  value.types = valueTypes;
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
  ::ad::map::landmark::LandmarkId valueTrafficLightId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.trafficLightId = valueTrafficLightId;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ContactLaneValidInputRangeTests, testValidInputRangeLocationTooSmall)
{
  ::ad::map::lane::ContactLane value;
  ::ad::map::lane::LaneId valueToLane(1);
  value.toLane = valueToLane;
  ::ad::map::lane::ContactLocation valueLocation(::ad::map::lane::ContactLocation::INVALID);
  value.location = valueLocation;
  ::ad::map::lane::ContactTypeList valueTypes;
  ::ad::map::lane::ContactType valueTypesElement(::ad::map::lane::ContactType::INVALID);
  valueTypes.resize(1, valueTypesElement);
  value.types = valueTypes;
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
  ::ad::map::landmark::LandmarkId valueTrafficLightId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.trafficLightId = valueTrafficLightId;

  // override member with data type value below input range minimum
  ::ad::map::lane::ContactLocation invalidInitializedMember(static_cast<::ad::map::lane::ContactLocation>(-1));
  value.location = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ContactLaneValidInputRangeTests, testValidInputRangeLocationTooBig)
{
  ::ad::map::lane::ContactLane value;
  ::ad::map::lane::LaneId valueToLane(1);
  value.toLane = valueToLane;
  ::ad::map::lane::ContactLocation valueLocation(::ad::map::lane::ContactLocation::INVALID);
  value.location = valueLocation;
  ::ad::map::lane::ContactTypeList valueTypes;
  ::ad::map::lane::ContactType valueTypesElement(::ad::map::lane::ContactType::INVALID);
  valueTypes.resize(1, valueTypesElement);
  value.types = valueTypes;
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
  ::ad::map::landmark::LandmarkId valueTrafficLightId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  value.trafficLightId = valueTrafficLightId;

  // override member with data type value above input range maximum
  ::ad::map::lane::ContactLocation invalidInitializedMember(static_cast<::ad::map::lane::ContactLocation>(-1));
  value.location = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
