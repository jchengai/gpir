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

#include "ad/map/restriction/RestrictionsValidInputRange.hpp"

TEST(RestrictionsValidInputRangeTests, testValidInputRange)
{
  ::ad::map::restriction::Restrictions value;
  ::ad::map::restriction::RestrictionList valueConjunctions;
  ::ad::map::restriction::Restriction valueConjunctionsElement;
  bool valueConjunctionsElementNegated{true};
  valueConjunctionsElement.negated = valueConjunctionsElementNegated;
  ::ad::map::restriction::RoadUserTypeList valueConjunctionsElementRoadUserTypes;
  ::ad::map::restriction::RoadUserType valueConjunctionsElementRoadUserTypesElement(
    ::ad::map::restriction::RoadUserType::INVALID);
  valueConjunctionsElementRoadUserTypes.resize(1, valueConjunctionsElementRoadUserTypesElement);
  valueConjunctionsElement.roadUserTypes = valueConjunctionsElementRoadUserTypes;
  ::ad::map::restriction::PassengerCount valueConjunctionsElementPassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
  valueConjunctionsElement.passengersMin = valueConjunctionsElementPassengersMin;
  valueConjunctions.resize(1, valueConjunctionsElement);
  value.conjunctions = valueConjunctions;
  ::ad::map::restriction::RestrictionList valueDisjunctions;
  ::ad::map::restriction::Restriction valueDisjunctionsElement;
  bool valueDisjunctionsElementNegated{true};
  valueDisjunctionsElement.negated = valueDisjunctionsElementNegated;
  ::ad::map::restriction::RoadUserTypeList valueDisjunctionsElementRoadUserTypes;
  ::ad::map::restriction::RoadUserType valueDisjunctionsElementRoadUserTypesElement(
    ::ad::map::restriction::RoadUserType::INVALID);
  valueDisjunctionsElementRoadUserTypes.resize(1, valueDisjunctionsElementRoadUserTypesElement);
  valueDisjunctionsElement.roadUserTypes = valueDisjunctionsElementRoadUserTypes;
  ::ad::map::restriction::PassengerCount valueDisjunctionsElementPassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
  valueDisjunctionsElement.passengersMin = valueDisjunctionsElementPassengersMin;
  valueDisjunctions.resize(1, valueDisjunctionsElement);
  value.disjunctions = valueDisjunctions;
  ASSERT_TRUE(withinValidInputRange(value));
}
