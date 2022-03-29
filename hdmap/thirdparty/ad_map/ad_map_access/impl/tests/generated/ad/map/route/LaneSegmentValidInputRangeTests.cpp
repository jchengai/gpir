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

#include "ad/map/route/LaneSegmentValidInputRange.hpp"

TEST(LaneSegmentValidInputRangeTests, testValidInputRange)
{
  ::ad::map::route::LaneSegment value;
  ::ad::map::lane::LaneId valueLeftNeighbor(1);
  value.leftNeighbor = valueLeftNeighbor;
  ::ad::map::lane::LaneId valueRightNeighbor(1);
  value.rightNeighbor = valueRightNeighbor;
  ::ad::map::lane::LaneIdList valuePredecessors;
  ::ad::map::lane::LaneId valuePredecessorsElement(1);
  valuePredecessors.resize(1, valuePredecessorsElement);
  value.predecessors = valuePredecessors;
  ::ad::map::lane::LaneIdList valueSuccessors;
  ::ad::map::lane::LaneId valueSuccessorsElement(1);
  valueSuccessors.resize(1, valueSuccessorsElement);
  value.successors = valueSuccessors;
  ::ad::map::route::LaneInterval valueLaneInterval;
  ::ad::map::lane::LaneId valueLaneIntervalLaneId(1);
  valueLaneInterval.laneId = valueLaneIntervalLaneId;
  ::ad::physics::ParametricValue valueLaneIntervalStart(0.);
  valueLaneInterval.start = valueLaneIntervalStart;
  ::ad::physics::ParametricValue valueLaneIntervalEnd(0.);
  valueLaneInterval.end = valueLaneIntervalEnd;
  bool valueLaneIntervalWrongWay{true};
  valueLaneInterval.wrongWay = valueLaneIntervalWrongWay;
  value.laneInterval = valueLaneInterval;
  ::ad::map::route::RouteLaneOffset valueRouteLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
  value.routeLaneOffset = valueRouteLaneOffset;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(LaneSegmentValidInputRangeTests, testValidInputRangeLaneIntervalTooSmall)
{
  ::ad::map::route::LaneSegment value;
  ::ad::map::lane::LaneId valueLeftNeighbor(1);
  value.leftNeighbor = valueLeftNeighbor;
  ::ad::map::lane::LaneId valueRightNeighbor(1);
  value.rightNeighbor = valueRightNeighbor;
  ::ad::map::lane::LaneIdList valuePredecessors;
  ::ad::map::lane::LaneId valuePredecessorsElement(1);
  valuePredecessors.resize(1, valuePredecessorsElement);
  value.predecessors = valuePredecessors;
  ::ad::map::lane::LaneIdList valueSuccessors;
  ::ad::map::lane::LaneId valueSuccessorsElement(1);
  valueSuccessors.resize(1, valueSuccessorsElement);
  value.successors = valueSuccessors;
  ::ad::map::route::LaneInterval valueLaneInterval;
  ::ad::map::lane::LaneId valueLaneIntervalLaneId(1);
  valueLaneInterval.laneId = valueLaneIntervalLaneId;
  ::ad::physics::ParametricValue valueLaneIntervalStart(0.);
  valueLaneInterval.start = valueLaneIntervalStart;
  ::ad::physics::ParametricValue valueLaneIntervalEnd(0.);
  valueLaneInterval.end = valueLaneIntervalEnd;
  bool valueLaneIntervalWrongWay{true};
  valueLaneInterval.wrongWay = valueLaneIntervalWrongWay;
  value.laneInterval = valueLaneInterval;
  ::ad::map::route::RouteLaneOffset valueRouteLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
  value.routeLaneOffset = valueRouteLaneOffset;

  // override member with data type value below input range minimum
  ::ad::map::route::LaneInterval invalidInitializedMember;
  ::ad::physics::ParametricValue invalidInitializedMemberStart(0. - ::ad::physics::ParametricValue::cPrecisionValue);
  invalidInitializedMember.start = invalidInitializedMemberStart;
  value.laneInterval = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(LaneSegmentValidInputRangeTests, testValidInputRangeLaneIntervalTooBig)
{
  ::ad::map::route::LaneSegment value;
  ::ad::map::lane::LaneId valueLeftNeighbor(1);
  value.leftNeighbor = valueLeftNeighbor;
  ::ad::map::lane::LaneId valueRightNeighbor(1);
  value.rightNeighbor = valueRightNeighbor;
  ::ad::map::lane::LaneIdList valuePredecessors;
  ::ad::map::lane::LaneId valuePredecessorsElement(1);
  valuePredecessors.resize(1, valuePredecessorsElement);
  value.predecessors = valuePredecessors;
  ::ad::map::lane::LaneIdList valueSuccessors;
  ::ad::map::lane::LaneId valueSuccessorsElement(1);
  valueSuccessors.resize(1, valueSuccessorsElement);
  value.successors = valueSuccessors;
  ::ad::map::route::LaneInterval valueLaneInterval;
  ::ad::map::lane::LaneId valueLaneIntervalLaneId(1);
  valueLaneInterval.laneId = valueLaneIntervalLaneId;
  ::ad::physics::ParametricValue valueLaneIntervalStart(0.);
  valueLaneInterval.start = valueLaneIntervalStart;
  ::ad::physics::ParametricValue valueLaneIntervalEnd(0.);
  valueLaneInterval.end = valueLaneIntervalEnd;
  bool valueLaneIntervalWrongWay{true};
  valueLaneInterval.wrongWay = valueLaneIntervalWrongWay;
  value.laneInterval = valueLaneInterval;
  ::ad::map::route::RouteLaneOffset valueRouteLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
  value.routeLaneOffset = valueRouteLaneOffset;

  // override member with data type value above input range maximum
  ::ad::map::route::LaneInterval invalidInitializedMember;
  ::ad::physics::ParametricValue invalidInitializedMemberStart(1. * 1.1);
  invalidInitializedMember.start = invalidInitializedMemberStart;
  value.laneInterval = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
