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

#include "ad/map/restriction/RestrictionListValidInputRange.hpp"

TEST(RestrictionListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::restriction::RestrictionList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(RestrictionListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::restriction::RestrictionList value;
  ::ad::map::restriction::Restriction element;
  bool elementNegated{true};
  element.negated = elementNegated;
  ::ad::map::restriction::RoadUserTypeList elementRoadUserTypes;
  ::ad::map::restriction::RoadUserType elementRoadUserTypesElement(::ad::map::restriction::RoadUserType::INVALID);
  elementRoadUserTypes.resize(1, elementRoadUserTypesElement);
  element.roadUserTypes = elementRoadUserTypes;
  ::ad::map::restriction::PassengerCount elementPassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
  element.passengersMin = elementPassengersMin;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}
