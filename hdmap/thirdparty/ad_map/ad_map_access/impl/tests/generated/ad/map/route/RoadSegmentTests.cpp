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
#include "ad/map/route/RoadSegment.hpp"

class RoadSegmentTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::route::RoadSegment value;
    ::ad::map::route::LaneSegmentList valueDrivableLaneSegments;
    ::ad::map::route::LaneSegment valueDrivableLaneSegmentsElement;
    ::ad::map::lane::LaneId valueDrivableLaneSegmentsElementLeftNeighbor(1);
    valueDrivableLaneSegmentsElement.leftNeighbor = valueDrivableLaneSegmentsElementLeftNeighbor;
    ::ad::map::lane::LaneId valueDrivableLaneSegmentsElementRightNeighbor(1);
    valueDrivableLaneSegmentsElement.rightNeighbor = valueDrivableLaneSegmentsElementRightNeighbor;
    ::ad::map::lane::LaneIdList valueDrivableLaneSegmentsElementPredecessors;
    ::ad::map::lane::LaneId valueDrivableLaneSegmentsElementPredecessorsElement(1);
    valueDrivableLaneSegmentsElementPredecessors.resize(1, valueDrivableLaneSegmentsElementPredecessorsElement);
    valueDrivableLaneSegmentsElement.predecessors = valueDrivableLaneSegmentsElementPredecessors;
    ::ad::map::lane::LaneIdList valueDrivableLaneSegmentsElementSuccessors;
    ::ad::map::lane::LaneId valueDrivableLaneSegmentsElementSuccessorsElement(1);
    valueDrivableLaneSegmentsElementSuccessors.resize(1, valueDrivableLaneSegmentsElementSuccessorsElement);
    valueDrivableLaneSegmentsElement.successors = valueDrivableLaneSegmentsElementSuccessors;
    ::ad::map::route::LaneInterval valueDrivableLaneSegmentsElementLaneInterval;
    ::ad::map::lane::LaneId valueDrivableLaneSegmentsElementLaneIntervalLaneId(1);
    valueDrivableLaneSegmentsElementLaneInterval.laneId = valueDrivableLaneSegmentsElementLaneIntervalLaneId;
    ::ad::physics::ParametricValue valueDrivableLaneSegmentsElementLaneIntervalStart(0.);
    valueDrivableLaneSegmentsElementLaneInterval.start = valueDrivableLaneSegmentsElementLaneIntervalStart;
    ::ad::physics::ParametricValue valueDrivableLaneSegmentsElementLaneIntervalEnd(0.);
    valueDrivableLaneSegmentsElementLaneInterval.end = valueDrivableLaneSegmentsElementLaneIntervalEnd;
    bool valueDrivableLaneSegmentsElementLaneIntervalWrongWay{true};
    valueDrivableLaneSegmentsElementLaneInterval.wrongWay = valueDrivableLaneSegmentsElementLaneIntervalWrongWay;
    valueDrivableLaneSegmentsElement.laneInterval = valueDrivableLaneSegmentsElementLaneInterval;
    ::ad::map::route::RouteLaneOffset valueDrivableLaneSegmentsElementRouteLaneOffset(
      std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
    valueDrivableLaneSegmentsElement.routeLaneOffset = valueDrivableLaneSegmentsElementRouteLaneOffset;
    valueDrivableLaneSegments.resize(1, valueDrivableLaneSegmentsElement);
    value.drivableLaneSegments = valueDrivableLaneSegments;
    ::ad::map::route::SegmentCounter valueSegmentCountFromDestination(
      std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
    value.segmentCountFromDestination = valueSegmentCountFromDestination;
    ::ad::map::point::BoundingSphere valueBoundingSphere;
    ::ad::map::point::ECEFPoint valueBoundingSphereCenter;
    ::ad::map::point::ECEFCoordinate valueBoundingSphereCenterX(-6400000);
    valueBoundingSphereCenter.x = valueBoundingSphereCenterX;
    ::ad::map::point::ECEFCoordinate valueBoundingSphereCenterY(-6400000);
    valueBoundingSphereCenter.y = valueBoundingSphereCenterY;
    ::ad::map::point::ECEFCoordinate valueBoundingSphereCenterZ(-6400000);
    valueBoundingSphereCenter.z = valueBoundingSphereCenterZ;
    valueBoundingSphere.center = valueBoundingSphereCenter;
    ::ad::physics::Distance valueBoundingSphereRadius(-1e9);
    valueBoundingSphere.radius = valueBoundingSphereRadius;
    value.boundingSphere = valueBoundingSphere;
    mValue = value;
  }

  ::ad::map::route::RoadSegment mValue;
};

TEST_F(RoadSegmentTests, copyConstruction)
{
  ::ad::map::route::RoadSegment value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(RoadSegmentTests, moveConstruction)
{
  ::ad::map::route::RoadSegment tmpValue(mValue);
  ::ad::map::route::RoadSegment value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(RoadSegmentTests, copyAssignment)
{
  ::ad::map::route::RoadSegment value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(RoadSegmentTests, moveAssignment)
{
  ::ad::map::route::RoadSegment tmpValue(mValue);
  ::ad::map::route::RoadSegment value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(RoadSegmentTests, comparisonOperatorEqual)
{
  ::ad::map::route::RoadSegment valueA = mValue;
  ::ad::map::route::RoadSegment valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(RoadSegmentTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(RoadSegmentTests, comparisonOperatorDrivableLaneSegmentsDiffers)
{
  ::ad::map::route::RoadSegment valueA = mValue;
  ::ad::map::route::LaneSegmentList drivableLaneSegments;
  ::ad::map::route::LaneSegment drivableLaneSegmentsElement;
  ::ad::map::lane::LaneId drivableLaneSegmentsElementLeftNeighbor(std::numeric_limits<::ad::map::lane::LaneId>::max());
  drivableLaneSegmentsElement.leftNeighbor = drivableLaneSegmentsElementLeftNeighbor;
  ::ad::map::lane::LaneId drivableLaneSegmentsElementRightNeighbor(std::numeric_limits<::ad::map::lane::LaneId>::max());
  drivableLaneSegmentsElement.rightNeighbor = drivableLaneSegmentsElementRightNeighbor;
  ::ad::map::lane::LaneIdList drivableLaneSegmentsElementPredecessors;
  ::ad::map::lane::LaneId drivableLaneSegmentsElementPredecessorsElement(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  drivableLaneSegmentsElementPredecessors.resize(2, drivableLaneSegmentsElementPredecessorsElement);
  drivableLaneSegmentsElement.predecessors = drivableLaneSegmentsElementPredecessors;
  ::ad::map::lane::LaneIdList drivableLaneSegmentsElementSuccessors;
  ::ad::map::lane::LaneId drivableLaneSegmentsElementSuccessorsElement(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  drivableLaneSegmentsElementSuccessors.resize(2, drivableLaneSegmentsElementSuccessorsElement);
  drivableLaneSegmentsElement.successors = drivableLaneSegmentsElementSuccessors;
  ::ad::map::route::LaneInterval drivableLaneSegmentsElementLaneInterval;
  ::ad::map::lane::LaneId drivableLaneSegmentsElementLaneIntervalLaneId(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  drivableLaneSegmentsElementLaneInterval.laneId = drivableLaneSegmentsElementLaneIntervalLaneId;
  ::ad::physics::ParametricValue drivableLaneSegmentsElementLaneIntervalStart(1.);
  drivableLaneSegmentsElementLaneInterval.start = drivableLaneSegmentsElementLaneIntervalStart;
  ::ad::physics::ParametricValue drivableLaneSegmentsElementLaneIntervalEnd(1.);
  drivableLaneSegmentsElementLaneInterval.end = drivableLaneSegmentsElementLaneIntervalEnd;
  bool drivableLaneSegmentsElementLaneIntervalWrongWay{false};
  drivableLaneSegmentsElementLaneInterval.wrongWay = drivableLaneSegmentsElementLaneIntervalWrongWay;
  drivableLaneSegmentsElement.laneInterval = drivableLaneSegmentsElementLaneInterval;
  ::ad::map::route::RouteLaneOffset drivableLaneSegmentsElementRouteLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  drivableLaneSegmentsElement.routeLaneOffset = drivableLaneSegmentsElementRouteLaneOffset;
  drivableLaneSegments.resize(2, drivableLaneSegmentsElement);
  valueA.drivableLaneSegments = drivableLaneSegments;
  ::ad::map::route::RoadSegment valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(RoadSegmentTests, comparisonOperatorSegmentCountFromDestinationDiffers)
{
  ::ad::map::route::RoadSegment valueA = mValue;
  ::ad::map::route::SegmentCounter segmentCountFromDestination(
    std::numeric_limits<::ad::map::route::SegmentCounter>::max());
  valueA.segmentCountFromDestination = segmentCountFromDestination;
  ::ad::map::route::RoadSegment valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(RoadSegmentTests, comparisonOperatorBoundingSphereDiffers)
{
  ::ad::map::route::RoadSegment valueA = mValue;
  ::ad::map::point::BoundingSphere boundingSphere;
  ::ad::map::point::ECEFPoint boundingSphereCenter;
  ::ad::map::point::ECEFCoordinate boundingSphereCenterX(6400000);
  boundingSphereCenter.x = boundingSphereCenterX;
  ::ad::map::point::ECEFCoordinate boundingSphereCenterY(6400000);
  boundingSphereCenter.y = boundingSphereCenterY;
  ::ad::map::point::ECEFCoordinate boundingSphereCenterZ(6400000);
  boundingSphereCenter.z = boundingSphereCenterZ;
  boundingSphere.center = boundingSphereCenter;
  ::ad::physics::Distance boundingSphereRadius(1e9);
  boundingSphere.radius = boundingSphereRadius;
  valueA.boundingSphere = boundingSphere;
  ::ad::map::route::RoadSegment valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
