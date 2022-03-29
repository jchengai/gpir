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

#include "ad/map/route/LaneSegmentListValidInputRange.hpp"

TEST(LaneSegmentListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::route::LaneSegmentList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LaneSegmentListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::route::LaneSegmentList value;
  ::ad::map::route::LaneSegment element;
  ::ad::map::lane::LaneId elementLeftNeighbor(1);
  element.leftNeighbor = elementLeftNeighbor;
  ::ad::map::lane::LaneId elementRightNeighbor(1);
  element.rightNeighbor = elementRightNeighbor;
  ::ad::map::lane::LaneIdList elementPredecessors;
  ::ad::map::lane::LaneId elementPredecessorsElement(1);
  elementPredecessors.resize(1, elementPredecessorsElement);
  element.predecessors = elementPredecessors;
  ::ad::map::lane::LaneIdList elementSuccessors;
  ::ad::map::lane::LaneId elementSuccessorsElement(1);
  elementSuccessors.resize(1, elementSuccessorsElement);
  element.successors = elementSuccessors;
  ::ad::map::route::LaneInterval elementLaneInterval;
  ::ad::map::lane::LaneId elementLaneIntervalLaneId(1);
  elementLaneInterval.laneId = elementLaneIntervalLaneId;
  ::ad::physics::ParametricValue elementLaneIntervalStart(0.);
  elementLaneInterval.start = elementLaneIntervalStart;
  ::ad::physics::ParametricValue elementLaneIntervalEnd(0.);
  elementLaneInterval.end = elementLaneIntervalEnd;
  bool elementLaneIntervalWrongWay{true};
  elementLaneInterval.wrongWay = elementLaneIntervalWrongWay;
  element.laneInterval = elementLaneInterval;
  ::ad::map::route::RouteLaneOffset elementRouteLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
  element.routeLaneOffset = elementRouteLaneOffset;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LaneSegmentListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::route::LaneSegmentList value;
  ::ad::map::route::LaneSegment element;
  ::ad::map::route::LaneInterval elementLaneInterval;
  ::ad::physics::ParametricValue elementLaneIntervalStart(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  elementLaneInterval.start = elementLaneIntervalStart;
  element.laneInterval = elementLaneInterval;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
