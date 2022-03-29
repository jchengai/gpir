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

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wself-assign-overloaded"
#endif

#include <gtest/gtest.h>
#include <limits>
#include "ad/map/route/LaneSegment.hpp"

class LaneSegmentTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
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
    mValue = value;
  }

  ::ad::map::route::LaneSegment mValue;
};

TEST_F(LaneSegmentTests, copyConstruction)
{
  ::ad::map::route::LaneSegment value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneSegmentTests, moveConstruction)
{
  ::ad::map::route::LaneSegment tmpValue(mValue);
  ::ad::map::route::LaneSegment value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneSegmentTests, copyAssignment)
{
  ::ad::map::route::LaneSegment value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneSegmentTests, moveAssignment)
{
  ::ad::map::route::LaneSegment tmpValue(mValue);
  ::ad::map::route::LaneSegment value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(LaneSegmentTests, comparisonOperatorEqual)
{
  ::ad::map::route::LaneSegment valueA = mValue;
  ::ad::map::route::LaneSegment valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(LaneSegmentTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(LaneSegmentTests, comparisonOperatorLeftNeighborDiffers)
{
  ::ad::map::route::LaneSegment valueA = mValue;
  ::ad::map::lane::LaneId leftNeighbor(std::numeric_limits<::ad::map::lane::LaneId>::max());
  valueA.leftNeighbor = leftNeighbor;
  ::ad::map::route::LaneSegment valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneSegmentTests, comparisonOperatorRightNeighborDiffers)
{
  ::ad::map::route::LaneSegment valueA = mValue;
  ::ad::map::lane::LaneId rightNeighbor(std::numeric_limits<::ad::map::lane::LaneId>::max());
  valueA.rightNeighbor = rightNeighbor;
  ::ad::map::route::LaneSegment valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneSegmentTests, comparisonOperatorPredecessorsDiffers)
{
  ::ad::map::route::LaneSegment valueA = mValue;
  ::ad::map::lane::LaneIdList predecessors;
  ::ad::map::lane::LaneId predecessorsElement(std::numeric_limits<::ad::map::lane::LaneId>::max());
  predecessors.resize(2, predecessorsElement);
  valueA.predecessors = predecessors;
  ::ad::map::route::LaneSegment valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneSegmentTests, comparisonOperatorSuccessorsDiffers)
{
  ::ad::map::route::LaneSegment valueA = mValue;
  ::ad::map::lane::LaneIdList successors;
  ::ad::map::lane::LaneId successorsElement(std::numeric_limits<::ad::map::lane::LaneId>::max());
  successors.resize(2, successorsElement);
  valueA.successors = successors;
  ::ad::map::route::LaneSegment valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneSegmentTests, comparisonOperatorLaneIntervalDiffers)
{
  ::ad::map::route::LaneSegment valueA = mValue;
  ::ad::map::route::LaneInterval laneInterval;
  ::ad::map::lane::LaneId laneIntervalLaneId(std::numeric_limits<::ad::map::lane::LaneId>::max());
  laneInterval.laneId = laneIntervalLaneId;
  ::ad::physics::ParametricValue laneIntervalStart(1.);
  laneInterval.start = laneIntervalStart;
  ::ad::physics::ParametricValue laneIntervalEnd(1.);
  laneInterval.end = laneIntervalEnd;
  bool laneIntervalWrongWay{false};
  laneInterval.wrongWay = laneIntervalWrongWay;
  valueA.laneInterval = laneInterval;
  ::ad::map::route::LaneSegment valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(LaneSegmentTests, comparisonOperatorRouteLaneOffsetDiffers)
{
  ::ad::map::route::LaneSegment valueA = mValue;
  ::ad::map::route::RouteLaneOffset routeLaneOffset(std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  valueA.routeLaneOffset = routeLaneOffset;
  ::ad::map::route::LaneSegment valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
