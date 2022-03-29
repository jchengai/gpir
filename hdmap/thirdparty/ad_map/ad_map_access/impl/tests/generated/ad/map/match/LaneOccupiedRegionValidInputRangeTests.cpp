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

#include "ad/map/match/LaneOccupiedRegionValidInputRange.hpp"

TEST(LaneOccupiedRegionValidInputRangeTests, testValidInputRange)
{
  ::ad::map::match::LaneOccupiedRegion value;
  ::ad::map::lane::LaneId valueLaneId(1);
  value.laneId = valueLaneId;
  ::ad::physics::ParametricRange valueLongitudinalRange;
  ::ad::physics::ParametricValue valueLongitudinalRangeMinimum(0.);
  valueLongitudinalRange.minimum = valueLongitudinalRangeMinimum;
  ::ad::physics::ParametricValue valueLongitudinalRangeMaximum(0.);
  valueLongitudinalRange.maximum = valueLongitudinalRangeMaximum;
  valueLongitudinalRange.maximum = valueLongitudinalRange.minimum;
  valueLongitudinalRange.minimum = valueLongitudinalRange.maximum;
  value.longitudinalRange = valueLongitudinalRange;
  ::ad::physics::ParametricRange valueLateralRange;
  ::ad::physics::ParametricValue valueLateralRangeMinimum(0.);
  valueLateralRange.minimum = valueLateralRangeMinimum;
  ::ad::physics::ParametricValue valueLateralRangeMaximum(0.);
  valueLateralRange.maximum = valueLateralRangeMaximum;
  valueLateralRange.maximum = valueLateralRange.minimum;
  valueLateralRange.minimum = valueLateralRange.maximum;
  value.lateralRange = valueLateralRange;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LaneOccupiedRegionValidInputRangeTests, testValidInputRangeLongitudinalRangeTooSmall)
{
  ::ad::map::match::LaneOccupiedRegion value;
  ::ad::map::lane::LaneId valueLaneId(1);
  value.laneId = valueLaneId;
  ::ad::physics::ParametricRange valueLongitudinalRange;
  ::ad::physics::ParametricValue valueLongitudinalRangeMinimum(0.);
  valueLongitudinalRange.minimum = valueLongitudinalRangeMinimum;
  ::ad::physics::ParametricValue valueLongitudinalRangeMaximum(0.);
  valueLongitudinalRange.maximum = valueLongitudinalRangeMaximum;
  valueLongitudinalRange.maximum = valueLongitudinalRange.minimum;
  valueLongitudinalRange.minimum = valueLongitudinalRange.maximum;
  value.longitudinalRange = valueLongitudinalRange;
  ::ad::physics::ParametricRange valueLateralRange;
  ::ad::physics::ParametricValue valueLateralRangeMinimum(0.);
  valueLateralRange.minimum = valueLateralRangeMinimum;
  ::ad::physics::ParametricValue valueLateralRangeMaximum(0.);
  valueLateralRange.maximum = valueLateralRangeMaximum;
  valueLateralRange.maximum = valueLateralRange.minimum;
  valueLateralRange.minimum = valueLateralRange.maximum;
  value.lateralRange = valueLateralRange;

  // override member with data type value below input range minimum
  ::ad::physics::ParametricRange invalidInitializedMember;
  ::ad::physics::ParametricValue invalidInitializedMemberMinimum(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  invalidInitializedMember.minimum = invalidInitializedMemberMinimum;
  value.longitudinalRange = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneOccupiedRegionValidInputRangeTests, testValidInputRangeLongitudinalRangeTooBig)
{
  ::ad::map::match::LaneOccupiedRegion value;
  ::ad::map::lane::LaneId valueLaneId(1);
  value.laneId = valueLaneId;
  ::ad::physics::ParametricRange valueLongitudinalRange;
  ::ad::physics::ParametricValue valueLongitudinalRangeMinimum(0.);
  valueLongitudinalRange.minimum = valueLongitudinalRangeMinimum;
  ::ad::physics::ParametricValue valueLongitudinalRangeMaximum(0.);
  valueLongitudinalRange.maximum = valueLongitudinalRangeMaximum;
  valueLongitudinalRange.maximum = valueLongitudinalRange.minimum;
  valueLongitudinalRange.minimum = valueLongitudinalRange.maximum;
  value.longitudinalRange = valueLongitudinalRange;
  ::ad::physics::ParametricRange valueLateralRange;
  ::ad::physics::ParametricValue valueLateralRangeMinimum(0.);
  valueLateralRange.minimum = valueLateralRangeMinimum;
  ::ad::physics::ParametricValue valueLateralRangeMaximum(0.);
  valueLateralRange.maximum = valueLateralRangeMaximum;
  valueLateralRange.maximum = valueLateralRange.minimum;
  valueLateralRange.minimum = valueLateralRange.maximum;
  value.lateralRange = valueLateralRange;

  // override member with data type value above input range maximum
  ::ad::physics::ParametricRange invalidInitializedMember;
  ::ad::physics::ParametricValue invalidInitializedMemberMinimum(1. * 1.1);
  invalidInitializedMember.minimum = invalidInitializedMemberMinimum;
  value.longitudinalRange = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneOccupiedRegionValidInputRangeTests, testValidInputRangeLateralRangeTooSmall)
{
  ::ad::map::match::LaneOccupiedRegion value;
  ::ad::map::lane::LaneId valueLaneId(1);
  value.laneId = valueLaneId;
  ::ad::physics::ParametricRange valueLongitudinalRange;
  ::ad::physics::ParametricValue valueLongitudinalRangeMinimum(0.);
  valueLongitudinalRange.minimum = valueLongitudinalRangeMinimum;
  ::ad::physics::ParametricValue valueLongitudinalRangeMaximum(0.);
  valueLongitudinalRange.maximum = valueLongitudinalRangeMaximum;
  valueLongitudinalRange.maximum = valueLongitudinalRange.minimum;
  valueLongitudinalRange.minimum = valueLongitudinalRange.maximum;
  value.longitudinalRange = valueLongitudinalRange;
  ::ad::physics::ParametricRange valueLateralRange;
  ::ad::physics::ParametricValue valueLateralRangeMinimum(0.);
  valueLateralRange.minimum = valueLateralRangeMinimum;
  ::ad::physics::ParametricValue valueLateralRangeMaximum(0.);
  valueLateralRange.maximum = valueLateralRangeMaximum;
  valueLateralRange.maximum = valueLateralRange.minimum;
  valueLateralRange.minimum = valueLateralRange.maximum;
  value.lateralRange = valueLateralRange;

  // override member with data type value below input range minimum
  ::ad::physics::ParametricRange invalidInitializedMember;
  ::ad::physics::ParametricValue invalidInitializedMemberMinimum(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  invalidInitializedMember.minimum = invalidInitializedMemberMinimum;
  value.lateralRange = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneOccupiedRegionValidInputRangeTests, testValidInputRangeLateralRangeTooBig)
{
  ::ad::map::match::LaneOccupiedRegion value;
  ::ad::map::lane::LaneId valueLaneId(1);
  value.laneId = valueLaneId;
  ::ad::physics::ParametricRange valueLongitudinalRange;
  ::ad::physics::ParametricValue valueLongitudinalRangeMinimum(0.);
  valueLongitudinalRange.minimum = valueLongitudinalRangeMinimum;
  ::ad::physics::ParametricValue valueLongitudinalRangeMaximum(0.);
  valueLongitudinalRange.maximum = valueLongitudinalRangeMaximum;
  valueLongitudinalRange.maximum = valueLongitudinalRange.minimum;
  valueLongitudinalRange.minimum = valueLongitudinalRange.maximum;
  value.longitudinalRange = valueLongitudinalRange;
  ::ad::physics::ParametricRange valueLateralRange;
  ::ad::physics::ParametricValue valueLateralRangeMinimum(0.);
  valueLateralRange.minimum = valueLateralRangeMinimum;
  ::ad::physics::ParametricValue valueLateralRangeMaximum(0.);
  valueLateralRange.maximum = valueLateralRangeMaximum;
  valueLateralRange.maximum = valueLateralRange.minimum;
  valueLateralRange.minimum = valueLateralRange.maximum;
  value.lateralRange = valueLateralRange;

  // override member with data type value above input range maximum
  ::ad::physics::ParametricRange invalidInitializedMember;
  ::ad::physics::ParametricValue invalidInitializedMemberMinimum(1. * 1.1);
  invalidInitializedMember.minimum = invalidInitializedMemberMinimum;
  value.lateralRange = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
