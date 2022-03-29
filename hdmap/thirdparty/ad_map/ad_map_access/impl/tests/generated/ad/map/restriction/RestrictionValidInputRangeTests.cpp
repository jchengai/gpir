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

#include "ad/map/restriction/RestrictionValidInputRange.hpp"

TEST(RestrictionValidInputRangeTests, testValidInputRange)
{
  ::ad::map::restriction::Restriction value;
  bool valueNegated{true};
  value.negated = valueNegated;
  ::ad::map::restriction::RoadUserTypeList valueRoadUserTypes;
  ::ad::map::restriction::RoadUserType valueRoadUserTypesElement(::ad::map::restriction::RoadUserType::INVALID);
  valueRoadUserTypes.resize(1, valueRoadUserTypesElement);
  value.roadUserTypes = valueRoadUserTypes;
  ::ad::map::restriction::PassengerCount valuePassengersMin(
    std::numeric_limits<::ad::map::restriction::PassengerCount>::lowest());
  value.passengersMin = valuePassengersMin;
  ASSERT_TRUE(withinValidInputRange(value));
}
