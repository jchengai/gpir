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

#include "ad/map/lane/ContactLaneListValidInputRange.hpp"

TEST(ContactLaneListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::lane::ContactLaneList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ContactLaneListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::lane::ContactLaneList value;
  ::ad::map::lane::ContactLane element;
  ::ad::map::lane::LaneId elementToLane(1);
  element.toLane = elementToLane;
  ::ad::map::lane::ContactLocation elementLocation(::ad::map::lane::ContactLocation::INVALID);
  element.location = elementLocation;
  ::ad::map::lane::ContactTypeList elementTypes;
  ::ad::map::lane::ContactType elementTypesElement(::ad::map::lane::ContactType::INVALID);
  elementTypes.resize(1, elementTypesElement);
  element.types = elementTypes;
  ::ad::map::restriction::Restrictions elementRestrictions;
  ::ad::map::restriction::RestrictionList elementRestrictionsConjunctions;
  ::ad::map::restriction::Restriction elementRestrictionsConjunctionsElement;
  bool elementRestrictionsConjunctionsElementNegated{true};
  elementRestrictionsConjunctionsElement.negated = elementRestrictionsConjunctionsElementNegated;
  ::ad::map::restriction::RoadUserTypeList elementRestrictionsConjunctionsElementRoadUserTypes;
  ::ad::map::restriction::RoadUserType elementRestrictionsConjunctionsElementRoadUserTypesElement(
    ::ad::map::restriction::RoadUserType::INVALID);
  elementRestrictionsConjunctionsElementRoadUserTypes.resize(
    1, elementRestrictionsConjunctionsElementRoadUserTypesElement);
  elementRestrictionsConjunctionsElement.roadUserTypes = elementRestrictionsConjunctionsElementRoadUserTypes;
  ::ad::map::restriction::PassengerCount elementRestrictionsConjunctionsElementPassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
  elementRestrictionsConjunctionsElement.passengersMin = elementRestrictionsConjunctionsElementPassengersMin;
  elementRestrictionsConjunctions.resize(1, elementRestrictionsConjunctionsElement);
  elementRestrictions.conjunctions = elementRestrictionsConjunctions;
  ::ad::map::restriction::RestrictionList elementRestrictionsDisjunctions;
  ::ad::map::restriction::Restriction elementRestrictionsDisjunctionsElement;
  bool elementRestrictionsDisjunctionsElementNegated{true};
  elementRestrictionsDisjunctionsElement.negated = elementRestrictionsDisjunctionsElementNegated;
  ::ad::map::restriction::RoadUserTypeList elementRestrictionsDisjunctionsElementRoadUserTypes;
  ::ad::map::restriction::RoadUserType elementRestrictionsDisjunctionsElementRoadUserTypesElement(
    ::ad::map::restriction::RoadUserType::INVALID);
  elementRestrictionsDisjunctionsElementRoadUserTypes.resize(
    1, elementRestrictionsDisjunctionsElementRoadUserTypesElement);
  elementRestrictionsDisjunctionsElement.roadUserTypes = elementRestrictionsDisjunctionsElementRoadUserTypes;
  ::ad::map::restriction::PassengerCount elementRestrictionsDisjunctionsElementPassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
  elementRestrictionsDisjunctionsElement.passengersMin = elementRestrictionsDisjunctionsElementPassengersMin;
  elementRestrictionsDisjunctions.resize(1, elementRestrictionsDisjunctionsElement);
  elementRestrictions.disjunctions = elementRestrictionsDisjunctions;
  element.restrictions = elementRestrictions;
  ::ad::map::landmark::LandmarkId elementTrafficLightId(std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest());
  element.trafficLightId = elementTrafficLightId;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ContactLaneListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::lane::ContactLaneList value;
  ::ad::map::lane::ContactLane element;
  ::ad::map::lane::ContactLocation elementLocation(static_cast<::ad::map::lane::ContactLocation>(-1));
  element.location = elementLocation;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
