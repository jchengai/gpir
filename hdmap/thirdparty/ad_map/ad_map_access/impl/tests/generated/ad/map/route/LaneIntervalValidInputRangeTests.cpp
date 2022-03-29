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

#include "ad/map/route/LaneIntervalValidInputRange.hpp"

TEST(LaneIntervalValidInputRangeTests, testValidInputRange)
{
  ::ad::map::route::LaneInterval value;
  ::ad::map::lane::LaneId valueLaneId(1);
  value.laneId = valueLaneId;
  ::ad::physics::ParametricValue valueStart(0.);
  value.start = valueStart;
  ::ad::physics::ParametricValue valueEnd(0.);
  value.end = valueEnd;
  bool valueWrongWay{true};
  value.wrongWay = valueWrongWay;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LaneIntervalValidInputRangeTests, testValidInputRangeStartTooSmall)
{
  ::ad::map::route::LaneInterval value;
  ::ad::map::lane::LaneId valueLaneId(1);
  value.laneId = valueLaneId;
  ::ad::physics::ParametricValue valueStart(0.);
  value.start = valueStart;
  ::ad::physics::ParametricValue valueEnd(0.);
  value.end = valueEnd;
  bool valueWrongWay{true};
  value.wrongWay = valueWrongWay;

  // override member with data type value below input range minimum
  ::ad::physics::ParametricValue invalidInitializedMember(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  value.start = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneIntervalValidInputRangeTests, testValidInputRangeStartTooBig)
{
  ::ad::map::route::LaneInterval value;
  ::ad::map::lane::LaneId valueLaneId(1);
  value.laneId = valueLaneId;
  ::ad::physics::ParametricValue valueStart(0.);
  value.start = valueStart;
  ::ad::physics::ParametricValue valueEnd(0.);
  value.end = valueEnd;
  bool valueWrongWay{true};
  value.wrongWay = valueWrongWay;

  // override member with data type value above input range maximum
  ::ad::physics::ParametricValue invalidInitializedMember(1. * 1.1);
  value.start = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneIntervalValidInputRangeTests, testValidInputRangestartDefault)
{
  ::ad::map::route::LaneInterval value;
  ::ad::physics::ParametricValue valueDefault;
  value.start = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneIntervalValidInputRangeTests, testValidInputRangeEndTooSmall)
{
  ::ad::map::route::LaneInterval value;
  ::ad::map::lane::LaneId valueLaneId(1);
  value.laneId = valueLaneId;
  ::ad::physics::ParametricValue valueStart(0.);
  value.start = valueStart;
  ::ad::physics::ParametricValue valueEnd(0.);
  value.end = valueEnd;
  bool valueWrongWay{true};
  value.wrongWay = valueWrongWay;

  // override member with data type value below input range minimum
  ::ad::physics::ParametricValue invalidInitializedMember(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  value.end = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneIntervalValidInputRangeTests, testValidInputRangeEndTooBig)
{
  ::ad::map::route::LaneInterval value;
  ::ad::map::lane::LaneId valueLaneId(1);
  value.laneId = valueLaneId;
  ::ad::physics::ParametricValue valueStart(0.);
  value.start = valueStart;
  ::ad::physics::ParametricValue valueEnd(0.);
  value.end = valueEnd;
  bool valueWrongWay{true};
  value.wrongWay = valueWrongWay;

  // override member with data type value above input range maximum
  ::ad::physics::ParametricValue invalidInitializedMember(1. * 1.1);
  value.end = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneIntervalValidInputRangeTests, testValidInputRangeendDefault)
{
  ::ad::map::route::LaneInterval value;
  ::ad::physics::ParametricValue valueDefault;
  value.end = valueDefault;
  ASSERT_FALSE(withinValidInputRange(value));
}
