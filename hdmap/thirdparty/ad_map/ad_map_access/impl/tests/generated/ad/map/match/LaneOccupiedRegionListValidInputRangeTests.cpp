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

#include "ad/map/match/LaneOccupiedRegionListValidInputRange.hpp"

TEST(LaneOccupiedRegionListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::match::LaneOccupiedRegionList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LaneOccupiedRegionListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::match::LaneOccupiedRegionList value;
  ::ad::map::match::LaneOccupiedRegion element;
  ::ad::map::lane::LaneId elementLaneId(1);
  element.laneId = elementLaneId;
  ::ad::physics::ParametricRange elementLongitudinalRange;
  ::ad::physics::ParametricValue elementLongitudinalRangeMinimum(0.);
  elementLongitudinalRange.minimum = elementLongitudinalRangeMinimum;
  ::ad::physics::ParametricValue elementLongitudinalRangeMaximum(0.);
  elementLongitudinalRange.maximum = elementLongitudinalRangeMaximum;
  elementLongitudinalRange.maximum = elementLongitudinalRange.minimum;
  elementLongitudinalRange.minimum = elementLongitudinalRange.maximum;
  element.longitudinalRange = elementLongitudinalRange;
  ::ad::physics::ParametricRange elementLateralRange;
  ::ad::physics::ParametricValue elementLateralRangeMinimum(0.);
  elementLateralRange.minimum = elementLateralRangeMinimum;
  ::ad::physics::ParametricValue elementLateralRangeMaximum(0.);
  elementLateralRange.maximum = elementLateralRangeMaximum;
  elementLateralRange.maximum = elementLateralRange.minimum;
  elementLateralRange.minimum = elementLateralRange.maximum;
  element.lateralRange = elementLateralRange;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LaneOccupiedRegionListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::match::LaneOccupiedRegionList value;
  ::ad::map::match::LaneOccupiedRegion element;
  ::ad::physics::ParametricRange elementLongitudinalRange;
  ::ad::physics::ParametricValue elementLongitudinalRangeMinimum(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  elementLongitudinalRange.minimum = elementLongitudinalRangeMinimum;
  element.longitudinalRange = elementLongitudinalRange;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
