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
#include "ad/map/route/FullRoute.hpp"

class FullRouteTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::route::FullRoute value;
    ::ad::map::route::RoadSegmentList valueRoadSegments;
    ::ad::map::route::RoadSegment valueRoadSegmentsElement;
    ::ad::map::route::LaneSegmentList valueRoadSegmentsElementDrivableLaneSegments;
    ::ad::map::route::LaneSegment valueRoadSegmentsElementDrivableLaneSegmentsElement;
    ::ad::map::lane::LaneId valueRoadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor(1);
    valueRoadSegmentsElementDrivableLaneSegmentsElement.leftNeighbor
      = valueRoadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor;
    ::ad::map::lane::LaneId valueRoadSegmentsElementDrivableLaneSegmentsElementRightNeighbor(1);
    valueRoadSegmentsElementDrivableLaneSegmentsElement.rightNeighbor
      = valueRoadSegmentsElementDrivableLaneSegmentsElementRightNeighbor;
    ::ad::map::lane::LaneIdList valueRoadSegmentsElementDrivableLaneSegmentsElementPredecessors;
    ::ad::map::lane::LaneId valueRoadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement(1);
    valueRoadSegmentsElementDrivableLaneSegmentsElementPredecessors.resize(
      1, valueRoadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement);
    valueRoadSegmentsElementDrivableLaneSegmentsElement.predecessors
      = valueRoadSegmentsElementDrivableLaneSegmentsElementPredecessors;
    ::ad::map::lane::LaneIdList valueRoadSegmentsElementDrivableLaneSegmentsElementSuccessors;
    ::ad::map::lane::LaneId valueRoadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement(1);
    valueRoadSegmentsElementDrivableLaneSegmentsElementSuccessors.resize(
      1, valueRoadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement);
    valueRoadSegmentsElementDrivableLaneSegmentsElement.successors
      = valueRoadSegmentsElementDrivableLaneSegmentsElementSuccessors;
    ::ad::map::route::LaneInterval valueRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
    ::ad::map::lane::LaneId valueRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId(1);
    valueRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.laneId
      = valueRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId;
    ::ad::physics::ParametricValue valueRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart(0.);
    valueRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.start
      = valueRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart;
    ::ad::physics::ParametricValue valueRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd(0.);
    valueRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.end
      = valueRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd;
    bool valueRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay{true};
    valueRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.wrongWay
      = valueRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay;
    valueRoadSegmentsElementDrivableLaneSegmentsElement.laneInterval
      = valueRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
    ::ad::map::route::RouteLaneOffset valueRoadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset(
      std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
    valueRoadSegmentsElementDrivableLaneSegmentsElement.routeLaneOffset
      = valueRoadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset;
    valueRoadSegmentsElementDrivableLaneSegments.resize(1, valueRoadSegmentsElementDrivableLaneSegmentsElement);
    valueRoadSegmentsElement.drivableLaneSegments = valueRoadSegmentsElementDrivableLaneSegments;
    ::ad::map::route::SegmentCounter valueRoadSegmentsElementSegmentCountFromDestination(
      std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
    valueRoadSegmentsElement.segmentCountFromDestination = valueRoadSegmentsElementSegmentCountFromDestination;
    ::ad::map::point::BoundingSphere valueRoadSegmentsElementBoundingSphere;
    ::ad::map::point::ECEFPoint valueRoadSegmentsElementBoundingSphereCenter;
    ::ad::map::point::ECEFCoordinate valueRoadSegmentsElementBoundingSphereCenterX(-6400000);
    valueRoadSegmentsElementBoundingSphereCenter.x = valueRoadSegmentsElementBoundingSphereCenterX;
    ::ad::map::point::ECEFCoordinate valueRoadSegmentsElementBoundingSphereCenterY(-6400000);
    valueRoadSegmentsElementBoundingSphereCenter.y = valueRoadSegmentsElementBoundingSphereCenterY;
    ::ad::map::point::ECEFCoordinate valueRoadSegmentsElementBoundingSphereCenterZ(-6400000);
    valueRoadSegmentsElementBoundingSphereCenter.z = valueRoadSegmentsElementBoundingSphereCenterZ;
    valueRoadSegmentsElementBoundingSphere.center = valueRoadSegmentsElementBoundingSphereCenter;
    ::ad::physics::Distance valueRoadSegmentsElementBoundingSphereRadius(-1e9);
    valueRoadSegmentsElementBoundingSphere.radius = valueRoadSegmentsElementBoundingSphereRadius;
    valueRoadSegmentsElement.boundingSphere = valueRoadSegmentsElementBoundingSphere;
    valueRoadSegments.resize(1, valueRoadSegmentsElement);
    value.roadSegments = valueRoadSegments;
    ::ad::map::route::RoutePlanningCounter valueRoutePlanningCounter(
      std::numeric_limits<::ad::map::route::RoutePlanningCounter>::lowest());
    value.routePlanningCounter = valueRoutePlanningCounter;
    ::ad::map::route::SegmentCounter valueFullRouteSegmentCount(
      std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
    value.fullRouteSegmentCount = valueFullRouteSegmentCount;
    ::ad::map::route::RouteLaneOffset valueDestinationLaneOffset(
      std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
    value.destinationLaneOffset = valueDestinationLaneOffset;
    ::ad::map::route::RouteLaneOffset valueMinLaneOffset(
      std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
    value.minLaneOffset = valueMinLaneOffset;
    ::ad::map::route::RouteLaneOffset valueMaxLaneOffset(
      std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
    value.maxLaneOffset = valueMaxLaneOffset;
    ::ad::map::route::RouteCreationMode valueRouteCreationMode(::ad::map::route::RouteCreationMode::Undefined);
    value.routeCreationMode = valueRouteCreationMode;
    mValue = value;
  }

  ::ad::map::route::FullRoute mValue;
};

TEST_F(FullRouteTests, copyConstruction)
{
  ::ad::map::route::FullRoute value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(FullRouteTests, moveConstruction)
{
  ::ad::map::route::FullRoute tmpValue(mValue);
  ::ad::map::route::FullRoute value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(FullRouteTests, copyAssignment)
{
  ::ad::map::route::FullRoute value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(FullRouteTests, moveAssignment)
{
  ::ad::map::route::FullRoute tmpValue(mValue);
  ::ad::map::route::FullRoute value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(FullRouteTests, comparisonOperatorEqual)
{
  ::ad::map::route::FullRoute valueA = mValue;
  ::ad::map::route::FullRoute valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(FullRouteTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(FullRouteTests, comparisonOperatorRoadSegmentsDiffers)
{
  ::ad::map::route::FullRoute valueA = mValue;
  ::ad::map::route::RoadSegmentList roadSegments;
  ::ad::map::route::RoadSegment roadSegmentsElement;
  ::ad::map::route::LaneSegmentList roadSegmentsElementDrivableLaneSegments;
  ::ad::map::route::LaneSegment roadSegmentsElementDrivableLaneSegmentsElement;
  ::ad::map::lane::LaneId roadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  roadSegmentsElementDrivableLaneSegmentsElement.leftNeighbor
    = roadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor;
  ::ad::map::lane::LaneId roadSegmentsElementDrivableLaneSegmentsElementRightNeighbor(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  roadSegmentsElementDrivableLaneSegmentsElement.rightNeighbor
    = roadSegmentsElementDrivableLaneSegmentsElementRightNeighbor;
  ::ad::map::lane::LaneIdList roadSegmentsElementDrivableLaneSegmentsElementPredecessors;
  ::ad::map::lane::LaneId roadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  roadSegmentsElementDrivableLaneSegmentsElementPredecessors.resize(
    2, roadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement);
  roadSegmentsElementDrivableLaneSegmentsElement.predecessors
    = roadSegmentsElementDrivableLaneSegmentsElementPredecessors;
  ::ad::map::lane::LaneIdList roadSegmentsElementDrivableLaneSegmentsElementSuccessors;
  ::ad::map::lane::LaneId roadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  roadSegmentsElementDrivableLaneSegmentsElementSuccessors.resize(
    2, roadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement);
  roadSegmentsElementDrivableLaneSegmentsElement.successors = roadSegmentsElementDrivableLaneSegmentsElementSuccessors;
  ::ad::map::route::LaneInterval roadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
  ::ad::map::lane::LaneId roadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  roadSegmentsElementDrivableLaneSegmentsElementLaneInterval.laneId
    = roadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId;
  ::ad::physics::ParametricValue roadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart(1.);
  roadSegmentsElementDrivableLaneSegmentsElementLaneInterval.start
    = roadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart;
  ::ad::physics::ParametricValue roadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd(1.);
  roadSegmentsElementDrivableLaneSegmentsElementLaneInterval.end
    = roadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd;
  bool roadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay{false};
  roadSegmentsElementDrivableLaneSegmentsElementLaneInterval.wrongWay
    = roadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay;
  roadSegmentsElementDrivableLaneSegmentsElement.laneInterval
    = roadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
  ::ad::map::route::RouteLaneOffset roadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  roadSegmentsElementDrivableLaneSegmentsElement.routeLaneOffset
    = roadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset;
  roadSegmentsElementDrivableLaneSegments.resize(2, roadSegmentsElementDrivableLaneSegmentsElement);
  roadSegmentsElement.drivableLaneSegments = roadSegmentsElementDrivableLaneSegments;
  ::ad::map::route::SegmentCounter roadSegmentsElementSegmentCountFromDestination(
    std::numeric_limits<::ad::map::route::SegmentCounter>::max());
  roadSegmentsElement.segmentCountFromDestination = roadSegmentsElementSegmentCountFromDestination;
  ::ad::map::point::BoundingSphere roadSegmentsElementBoundingSphere;
  ::ad::map::point::ECEFPoint roadSegmentsElementBoundingSphereCenter;
  ::ad::map::point::ECEFCoordinate roadSegmentsElementBoundingSphereCenterX(6400000);
  roadSegmentsElementBoundingSphereCenter.x = roadSegmentsElementBoundingSphereCenterX;
  ::ad::map::point::ECEFCoordinate roadSegmentsElementBoundingSphereCenterY(6400000);
  roadSegmentsElementBoundingSphereCenter.y = roadSegmentsElementBoundingSphereCenterY;
  ::ad::map::point::ECEFCoordinate roadSegmentsElementBoundingSphereCenterZ(6400000);
  roadSegmentsElementBoundingSphereCenter.z = roadSegmentsElementBoundingSphereCenterZ;
  roadSegmentsElementBoundingSphere.center = roadSegmentsElementBoundingSphereCenter;
  ::ad::physics::Distance roadSegmentsElementBoundingSphereRadius(1e9);
  roadSegmentsElementBoundingSphere.radius = roadSegmentsElementBoundingSphereRadius;
  roadSegmentsElement.boundingSphere = roadSegmentsElementBoundingSphere;
  roadSegments.resize(2, roadSegmentsElement);
  valueA.roadSegments = roadSegments;
  ::ad::map::route::FullRoute valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(FullRouteTests, comparisonOperatorRoutePlanningCounterDiffers)
{
  ::ad::map::route::FullRoute valueA = mValue;
  ::ad::map::route::RoutePlanningCounter routePlanningCounter(
    std::numeric_limits<::ad::map::route::RoutePlanningCounter>::max());
  valueA.routePlanningCounter = routePlanningCounter;
  ::ad::map::route::FullRoute valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(FullRouteTests, comparisonOperatorFullRouteSegmentCountDiffers)
{
  ::ad::map::route::FullRoute valueA = mValue;
  ::ad::map::route::SegmentCounter fullRouteSegmentCount(std::numeric_limits<::ad::map::route::SegmentCounter>::max());
  valueA.fullRouteSegmentCount = fullRouteSegmentCount;
  ::ad::map::route::FullRoute valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(FullRouteTests, comparisonOperatorDestinationLaneOffsetDiffers)
{
  ::ad::map::route::FullRoute valueA = mValue;
  ::ad::map::route::RouteLaneOffset destinationLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  valueA.destinationLaneOffset = destinationLaneOffset;
  ::ad::map::route::FullRoute valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(FullRouteTests, comparisonOperatorMinLaneOffsetDiffers)
{
  ::ad::map::route::FullRoute valueA = mValue;
  ::ad::map::route::RouteLaneOffset minLaneOffset(std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  valueA.minLaneOffset = minLaneOffset;
  ::ad::map::route::FullRoute valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(FullRouteTests, comparisonOperatorMaxLaneOffsetDiffers)
{
  ::ad::map::route::FullRoute valueA = mValue;
  ::ad::map::route::RouteLaneOffset maxLaneOffset(std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  valueA.maxLaneOffset = maxLaneOffset;
  ::ad::map::route::FullRoute valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(FullRouteTests, comparisonOperatorRouteCreationModeDiffers)
{
  ::ad::map::route::FullRoute valueA = mValue;
  ::ad::map::route::RouteCreationMode routeCreationMode(::ad::map::route::RouteCreationMode::AllNeighborLanes);
  valueA.routeCreationMode = routeCreationMode;
  ::ad::map::route::FullRoute valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
