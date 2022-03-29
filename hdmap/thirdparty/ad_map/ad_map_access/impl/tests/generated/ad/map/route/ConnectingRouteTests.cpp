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
#include "ad/map/route/ConnectingRoute.hpp"

class ConnectingRouteTests : public testing::Test
{
protected:
  virtual void SetUp() override
  {
    // valid initialization
    ::ad::map::route::ConnectingRoute value;
    ::ad::map::route::ConnectingRouteType valueType(::ad::map::route::ConnectingRouteType::Invalid);
    value.type = valueType;
    ::ad::map::route::FullRoute valueRouteA;
    ::ad::map::route::RoadSegmentList valueRouteARoadSegments;
    ::ad::map::route::RoadSegment valueRouteARoadSegmentsElement;
    ::ad::map::route::LaneSegmentList valueRouteARoadSegmentsElementDrivableLaneSegments;
    ::ad::map::route::LaneSegment valueRouteARoadSegmentsElementDrivableLaneSegmentsElement;
    ::ad::map::lane::LaneId valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor(1);
    valueRouteARoadSegmentsElementDrivableLaneSegmentsElement.leftNeighbor
      = valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor;
    ::ad::map::lane::LaneId valueRouteARoadSegmentsElementDrivableLaneSegmentsElementRightNeighbor(1);
    valueRouteARoadSegmentsElementDrivableLaneSegmentsElement.rightNeighbor
      = valueRouteARoadSegmentsElementDrivableLaneSegmentsElementRightNeighbor;
    ::ad::map::lane::LaneIdList valueRouteARoadSegmentsElementDrivableLaneSegmentsElementPredecessors;
    ::ad::map::lane::LaneId valueRouteARoadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement(1);
    valueRouteARoadSegmentsElementDrivableLaneSegmentsElementPredecessors.resize(
      1, valueRouteARoadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement);
    valueRouteARoadSegmentsElementDrivableLaneSegmentsElement.predecessors
      = valueRouteARoadSegmentsElementDrivableLaneSegmentsElementPredecessors;
    ::ad::map::lane::LaneIdList valueRouteARoadSegmentsElementDrivableLaneSegmentsElementSuccessors;
    ::ad::map::lane::LaneId valueRouteARoadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement(1);
    valueRouteARoadSegmentsElementDrivableLaneSegmentsElementSuccessors.resize(
      1, valueRouteARoadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement);
    valueRouteARoadSegmentsElementDrivableLaneSegmentsElement.successors
      = valueRouteARoadSegmentsElementDrivableLaneSegmentsElementSuccessors;
    ::ad::map::route::LaneInterval valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
    ::ad::map::lane::LaneId valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId(1);
    valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.laneId
      = valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId;
    ::ad::physics::ParametricValue valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart(0.);
    valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.start
      = valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart;
    ::ad::physics::ParametricValue valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd(0.);
    valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.end
      = valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd;
    bool valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay{true};
    valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.wrongWay
      = valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay;
    valueRouteARoadSegmentsElementDrivableLaneSegmentsElement.laneInterval
      = valueRouteARoadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
    ::ad::map::route::RouteLaneOffset valueRouteARoadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset(
      std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
    valueRouteARoadSegmentsElementDrivableLaneSegmentsElement.routeLaneOffset
      = valueRouteARoadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset;
    valueRouteARoadSegmentsElementDrivableLaneSegments.resize(
      1, valueRouteARoadSegmentsElementDrivableLaneSegmentsElement);
    valueRouteARoadSegmentsElement.drivableLaneSegments = valueRouteARoadSegmentsElementDrivableLaneSegments;
    ::ad::map::route::SegmentCounter valueRouteARoadSegmentsElementSegmentCountFromDestination(
      std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
    valueRouteARoadSegmentsElement.segmentCountFromDestination
      = valueRouteARoadSegmentsElementSegmentCountFromDestination;
    ::ad::map::point::BoundingSphere valueRouteARoadSegmentsElementBoundingSphere;
    ::ad::map::point::ECEFPoint valueRouteARoadSegmentsElementBoundingSphereCenter;
    ::ad::map::point::ECEFCoordinate valueRouteARoadSegmentsElementBoundingSphereCenterX(-6400000);
    valueRouteARoadSegmentsElementBoundingSphereCenter.x = valueRouteARoadSegmentsElementBoundingSphereCenterX;
    ::ad::map::point::ECEFCoordinate valueRouteARoadSegmentsElementBoundingSphereCenterY(-6400000);
    valueRouteARoadSegmentsElementBoundingSphereCenter.y = valueRouteARoadSegmentsElementBoundingSphereCenterY;
    ::ad::map::point::ECEFCoordinate valueRouteARoadSegmentsElementBoundingSphereCenterZ(-6400000);
    valueRouteARoadSegmentsElementBoundingSphereCenter.z = valueRouteARoadSegmentsElementBoundingSphereCenterZ;
    valueRouteARoadSegmentsElementBoundingSphere.center = valueRouteARoadSegmentsElementBoundingSphereCenter;
    ::ad::physics::Distance valueRouteARoadSegmentsElementBoundingSphereRadius(-1e9);
    valueRouteARoadSegmentsElementBoundingSphere.radius = valueRouteARoadSegmentsElementBoundingSphereRadius;
    valueRouteARoadSegmentsElement.boundingSphere = valueRouteARoadSegmentsElementBoundingSphere;
    valueRouteARoadSegments.resize(1, valueRouteARoadSegmentsElement);
    valueRouteA.roadSegments = valueRouteARoadSegments;
    ::ad::map::route::RoutePlanningCounter valueRouteARoutePlanningCounter(
      std::numeric_limits<::ad::map::route::RoutePlanningCounter>::lowest());
    valueRouteA.routePlanningCounter = valueRouteARoutePlanningCounter;
    ::ad::map::route::SegmentCounter valueRouteAFullRouteSegmentCount(
      std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
    valueRouteA.fullRouteSegmentCount = valueRouteAFullRouteSegmentCount;
    ::ad::map::route::RouteLaneOffset valueRouteADestinationLaneOffset(
      std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
    valueRouteA.destinationLaneOffset = valueRouteADestinationLaneOffset;
    ::ad::map::route::RouteLaneOffset valueRouteAMinLaneOffset(
      std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
    valueRouteA.minLaneOffset = valueRouteAMinLaneOffset;
    ::ad::map::route::RouteLaneOffset valueRouteAMaxLaneOffset(
      std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
    valueRouteA.maxLaneOffset = valueRouteAMaxLaneOffset;
    ::ad::map::route::RouteCreationMode valueRouteARouteCreationMode(::ad::map::route::RouteCreationMode::Undefined);
    valueRouteA.routeCreationMode = valueRouteARouteCreationMode;
    value.routeA = valueRouteA;
    ::ad::map::route::FullRoute valueRouteB;
    ::ad::map::route::RoadSegmentList valueRouteBRoadSegments;
    ::ad::map::route::RoadSegment valueRouteBRoadSegmentsElement;
    ::ad::map::route::LaneSegmentList valueRouteBRoadSegmentsElementDrivableLaneSegments;
    ::ad::map::route::LaneSegment valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement;
    ::ad::map::lane::LaneId valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor(1);
    valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement.leftNeighbor
      = valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor;
    ::ad::map::lane::LaneId valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementRightNeighbor(1);
    valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement.rightNeighbor
      = valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementRightNeighbor;
    ::ad::map::lane::LaneIdList valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementPredecessors;
    ::ad::map::lane::LaneId valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement(1);
    valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementPredecessors.resize(
      1, valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement);
    valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement.predecessors
      = valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementPredecessors;
    ::ad::map::lane::LaneIdList valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementSuccessors;
    ::ad::map::lane::LaneId valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement(1);
    valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementSuccessors.resize(
      1, valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement);
    valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement.successors
      = valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementSuccessors;
    ::ad::map::route::LaneInterval valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
    ::ad::map::lane::LaneId valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId(1);
    valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.laneId
      = valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId;
    ::ad::physics::ParametricValue valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart(0.);
    valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.start
      = valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart;
    ::ad::physics::ParametricValue valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd(0.);
    valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.end
      = valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd;
    bool valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay{true};
    valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.wrongWay
      = valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay;
    valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement.laneInterval
      = valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
    ::ad::map::route::RouteLaneOffset valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset(
      std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
    valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement.routeLaneOffset
      = valueRouteBRoadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset;
    valueRouteBRoadSegmentsElementDrivableLaneSegments.resize(
      1, valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement);
    valueRouteBRoadSegmentsElement.drivableLaneSegments = valueRouteBRoadSegmentsElementDrivableLaneSegments;
    ::ad::map::route::SegmentCounter valueRouteBRoadSegmentsElementSegmentCountFromDestination(
      std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
    valueRouteBRoadSegmentsElement.segmentCountFromDestination
      = valueRouteBRoadSegmentsElementSegmentCountFromDestination;
    ::ad::map::point::BoundingSphere valueRouteBRoadSegmentsElementBoundingSphere;
    ::ad::map::point::ECEFPoint valueRouteBRoadSegmentsElementBoundingSphereCenter;
    ::ad::map::point::ECEFCoordinate valueRouteBRoadSegmentsElementBoundingSphereCenterX(-6400000);
    valueRouteBRoadSegmentsElementBoundingSphereCenter.x = valueRouteBRoadSegmentsElementBoundingSphereCenterX;
    ::ad::map::point::ECEFCoordinate valueRouteBRoadSegmentsElementBoundingSphereCenterY(-6400000);
    valueRouteBRoadSegmentsElementBoundingSphereCenter.y = valueRouteBRoadSegmentsElementBoundingSphereCenterY;
    ::ad::map::point::ECEFCoordinate valueRouteBRoadSegmentsElementBoundingSphereCenterZ(-6400000);
    valueRouteBRoadSegmentsElementBoundingSphereCenter.z = valueRouteBRoadSegmentsElementBoundingSphereCenterZ;
    valueRouteBRoadSegmentsElementBoundingSphere.center = valueRouteBRoadSegmentsElementBoundingSphereCenter;
    ::ad::physics::Distance valueRouteBRoadSegmentsElementBoundingSphereRadius(-1e9);
    valueRouteBRoadSegmentsElementBoundingSphere.radius = valueRouteBRoadSegmentsElementBoundingSphereRadius;
    valueRouteBRoadSegmentsElement.boundingSphere = valueRouteBRoadSegmentsElementBoundingSphere;
    valueRouteBRoadSegments.resize(1, valueRouteBRoadSegmentsElement);
    valueRouteB.roadSegments = valueRouteBRoadSegments;
    ::ad::map::route::RoutePlanningCounter valueRouteBRoutePlanningCounter(
      std::numeric_limits<::ad::map::route::RoutePlanningCounter>::lowest());
    valueRouteB.routePlanningCounter = valueRouteBRoutePlanningCounter;
    ::ad::map::route::SegmentCounter valueRouteBFullRouteSegmentCount(
      std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
    valueRouteB.fullRouteSegmentCount = valueRouteBFullRouteSegmentCount;
    ::ad::map::route::RouteLaneOffset valueRouteBDestinationLaneOffset(
      std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
    valueRouteB.destinationLaneOffset = valueRouteBDestinationLaneOffset;
    ::ad::map::route::RouteLaneOffset valueRouteBMinLaneOffset(
      std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
    valueRouteB.minLaneOffset = valueRouteBMinLaneOffset;
    ::ad::map::route::RouteLaneOffset valueRouteBMaxLaneOffset(
      std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
    valueRouteB.maxLaneOffset = valueRouteBMaxLaneOffset;
    ::ad::map::route::RouteCreationMode valueRouteBRouteCreationMode(::ad::map::route::RouteCreationMode::Undefined);
    valueRouteB.routeCreationMode = valueRouteBRouteCreationMode;
    value.routeB = valueRouteB;
    mValue = value;
  }

  ::ad::map::route::ConnectingRoute mValue;
};

TEST_F(ConnectingRouteTests, copyConstruction)
{
  ::ad::map::route::ConnectingRoute value(mValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ConnectingRouteTests, moveConstruction)
{
  ::ad::map::route::ConnectingRoute tmpValue(mValue);
  ::ad::map::route::ConnectingRoute value(std::move(tmpValue));
  EXPECT_EQ(mValue, value);
}

TEST_F(ConnectingRouteTests, copyAssignment)
{
  ::ad::map::route::ConnectingRoute value;
  value = mValue;
  EXPECT_EQ(mValue, value);
}

TEST_F(ConnectingRouteTests, moveAssignment)
{
  ::ad::map::route::ConnectingRoute tmpValue(mValue);
  ::ad::map::route::ConnectingRoute value;
  value = std::move(tmpValue);
  EXPECT_EQ(mValue, value);
}

TEST_F(ConnectingRouteTests, comparisonOperatorEqual)
{
  ::ad::map::route::ConnectingRoute valueA = mValue;
  ::ad::map::route::ConnectingRoute valueB = mValue;

  EXPECT_TRUE(valueA == valueB);
  EXPECT_FALSE(valueA != valueB);
}

TEST_F(ConnectingRouteTests, stringConversionTest)
{
  std::stringstream stream;
  stream << mValue;
  std::string ostreamStr = stream.str();
  std::string toStr = std::to_string(mValue);
  ASSERT_EQ(ostreamStr, toStr);
}

TEST_F(ConnectingRouteTests, comparisonOperatorTypeDiffers)
{
  ::ad::map::route::ConnectingRoute valueA = mValue;
  ::ad::map::route::ConnectingRouteType type(::ad::map::route::ConnectingRouteType::Merging);
  valueA.type = type;
  ::ad::map::route::ConnectingRoute valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ConnectingRouteTests, comparisonOperatorRouteADiffers)
{
  ::ad::map::route::ConnectingRoute valueA = mValue;
  ::ad::map::route::FullRoute routeA;
  ::ad::map::route::RoadSegmentList routeARoadSegments;
  ::ad::map::route::RoadSegment routeARoadSegmentsElement;
  ::ad::map::route::LaneSegmentList routeARoadSegmentsElementDrivableLaneSegments;
  ::ad::map::route::LaneSegment routeARoadSegmentsElementDrivableLaneSegmentsElement;
  ::ad::map::lane::LaneId routeARoadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  routeARoadSegmentsElementDrivableLaneSegmentsElement.leftNeighbor
    = routeARoadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor;
  ::ad::map::lane::LaneId routeARoadSegmentsElementDrivableLaneSegmentsElementRightNeighbor(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  routeARoadSegmentsElementDrivableLaneSegmentsElement.rightNeighbor
    = routeARoadSegmentsElementDrivableLaneSegmentsElementRightNeighbor;
  ::ad::map::lane::LaneIdList routeARoadSegmentsElementDrivableLaneSegmentsElementPredecessors;
  ::ad::map::lane::LaneId routeARoadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  routeARoadSegmentsElementDrivableLaneSegmentsElementPredecessors.resize(
    2, routeARoadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement);
  routeARoadSegmentsElementDrivableLaneSegmentsElement.predecessors
    = routeARoadSegmentsElementDrivableLaneSegmentsElementPredecessors;
  ::ad::map::lane::LaneIdList routeARoadSegmentsElementDrivableLaneSegmentsElementSuccessors;
  ::ad::map::lane::LaneId routeARoadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  routeARoadSegmentsElementDrivableLaneSegmentsElementSuccessors.resize(
    2, routeARoadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement);
  routeARoadSegmentsElementDrivableLaneSegmentsElement.successors
    = routeARoadSegmentsElementDrivableLaneSegmentsElementSuccessors;
  ::ad::map::route::LaneInterval routeARoadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
  ::ad::map::lane::LaneId routeARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  routeARoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.laneId
    = routeARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId;
  ::ad::physics::ParametricValue routeARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart(1.);
  routeARoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.start
    = routeARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart;
  ::ad::physics::ParametricValue routeARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd(1.);
  routeARoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.end
    = routeARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd;
  bool routeARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay{false};
  routeARoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.wrongWay
    = routeARoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay;
  routeARoadSegmentsElementDrivableLaneSegmentsElement.laneInterval
    = routeARoadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
  ::ad::map::route::RouteLaneOffset routeARoadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  routeARoadSegmentsElementDrivableLaneSegmentsElement.routeLaneOffset
    = routeARoadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset;
  routeARoadSegmentsElementDrivableLaneSegments.resize(2, routeARoadSegmentsElementDrivableLaneSegmentsElement);
  routeARoadSegmentsElement.drivableLaneSegments = routeARoadSegmentsElementDrivableLaneSegments;
  ::ad::map::route::SegmentCounter routeARoadSegmentsElementSegmentCountFromDestination(
    std::numeric_limits<::ad::map::route::SegmentCounter>::max());
  routeARoadSegmentsElement.segmentCountFromDestination = routeARoadSegmentsElementSegmentCountFromDestination;
  ::ad::map::point::BoundingSphere routeARoadSegmentsElementBoundingSphere;
  ::ad::map::point::ECEFPoint routeARoadSegmentsElementBoundingSphereCenter;
  ::ad::map::point::ECEFCoordinate routeARoadSegmentsElementBoundingSphereCenterX(6400000);
  routeARoadSegmentsElementBoundingSphereCenter.x = routeARoadSegmentsElementBoundingSphereCenterX;
  ::ad::map::point::ECEFCoordinate routeARoadSegmentsElementBoundingSphereCenterY(6400000);
  routeARoadSegmentsElementBoundingSphereCenter.y = routeARoadSegmentsElementBoundingSphereCenterY;
  ::ad::map::point::ECEFCoordinate routeARoadSegmentsElementBoundingSphereCenterZ(6400000);
  routeARoadSegmentsElementBoundingSphereCenter.z = routeARoadSegmentsElementBoundingSphereCenterZ;
  routeARoadSegmentsElementBoundingSphere.center = routeARoadSegmentsElementBoundingSphereCenter;
  ::ad::physics::Distance routeARoadSegmentsElementBoundingSphereRadius(1e9);
  routeARoadSegmentsElementBoundingSphere.radius = routeARoadSegmentsElementBoundingSphereRadius;
  routeARoadSegmentsElement.boundingSphere = routeARoadSegmentsElementBoundingSphere;
  routeARoadSegments.resize(2, routeARoadSegmentsElement);
  routeA.roadSegments = routeARoadSegments;
  ::ad::map::route::RoutePlanningCounter routeARoutePlanningCounter(
    std::numeric_limits<::ad::map::route::RoutePlanningCounter>::max());
  routeA.routePlanningCounter = routeARoutePlanningCounter;
  ::ad::map::route::SegmentCounter routeAFullRouteSegmentCount(
    std::numeric_limits<::ad::map::route::SegmentCounter>::max());
  routeA.fullRouteSegmentCount = routeAFullRouteSegmentCount;
  ::ad::map::route::RouteLaneOffset routeADestinationLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  routeA.destinationLaneOffset = routeADestinationLaneOffset;
  ::ad::map::route::RouteLaneOffset routeAMinLaneOffset(std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  routeA.minLaneOffset = routeAMinLaneOffset;
  ::ad::map::route::RouteLaneOffset routeAMaxLaneOffset(std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  routeA.maxLaneOffset = routeAMaxLaneOffset;
  ::ad::map::route::RouteCreationMode routeARouteCreationMode(::ad::map::route::RouteCreationMode::AllNeighborLanes);
  routeA.routeCreationMode = routeARouteCreationMode;
  valueA.routeA = routeA;
  ::ad::map::route::ConnectingRoute valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

TEST_F(ConnectingRouteTests, comparisonOperatorRouteBDiffers)
{
  ::ad::map::route::ConnectingRoute valueA = mValue;
  ::ad::map::route::FullRoute routeB;
  ::ad::map::route::RoadSegmentList routeBRoadSegments;
  ::ad::map::route::RoadSegment routeBRoadSegmentsElement;
  ::ad::map::route::LaneSegmentList routeBRoadSegmentsElementDrivableLaneSegments;
  ::ad::map::route::LaneSegment routeBRoadSegmentsElementDrivableLaneSegmentsElement;
  ::ad::map::lane::LaneId routeBRoadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  routeBRoadSegmentsElementDrivableLaneSegmentsElement.leftNeighbor
    = routeBRoadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor;
  ::ad::map::lane::LaneId routeBRoadSegmentsElementDrivableLaneSegmentsElementRightNeighbor(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  routeBRoadSegmentsElementDrivableLaneSegmentsElement.rightNeighbor
    = routeBRoadSegmentsElementDrivableLaneSegmentsElementRightNeighbor;
  ::ad::map::lane::LaneIdList routeBRoadSegmentsElementDrivableLaneSegmentsElementPredecessors;
  ::ad::map::lane::LaneId routeBRoadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  routeBRoadSegmentsElementDrivableLaneSegmentsElementPredecessors.resize(
    2, routeBRoadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement);
  routeBRoadSegmentsElementDrivableLaneSegmentsElement.predecessors
    = routeBRoadSegmentsElementDrivableLaneSegmentsElementPredecessors;
  ::ad::map::lane::LaneIdList routeBRoadSegmentsElementDrivableLaneSegmentsElementSuccessors;
  ::ad::map::lane::LaneId routeBRoadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  routeBRoadSegmentsElementDrivableLaneSegmentsElementSuccessors.resize(
    2, routeBRoadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement);
  routeBRoadSegmentsElementDrivableLaneSegmentsElement.successors
    = routeBRoadSegmentsElementDrivableLaneSegmentsElementSuccessors;
  ::ad::map::route::LaneInterval routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
  ::ad::map::lane::LaneId routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId(
    std::numeric_limits<::ad::map::lane::LaneId>::max());
  routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.laneId
    = routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId;
  ::ad::physics::ParametricValue routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart(1.);
  routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.start
    = routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart;
  ::ad::physics::ParametricValue routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd(1.);
  routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.end
    = routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd;
  bool routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay{false};
  routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.wrongWay
    = routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay;
  routeBRoadSegmentsElementDrivableLaneSegmentsElement.laneInterval
    = routeBRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
  ::ad::map::route::RouteLaneOffset routeBRoadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  routeBRoadSegmentsElementDrivableLaneSegmentsElement.routeLaneOffset
    = routeBRoadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset;
  routeBRoadSegmentsElementDrivableLaneSegments.resize(2, routeBRoadSegmentsElementDrivableLaneSegmentsElement);
  routeBRoadSegmentsElement.drivableLaneSegments = routeBRoadSegmentsElementDrivableLaneSegments;
  ::ad::map::route::SegmentCounter routeBRoadSegmentsElementSegmentCountFromDestination(
    std::numeric_limits<::ad::map::route::SegmentCounter>::max());
  routeBRoadSegmentsElement.segmentCountFromDestination = routeBRoadSegmentsElementSegmentCountFromDestination;
  ::ad::map::point::BoundingSphere routeBRoadSegmentsElementBoundingSphere;
  ::ad::map::point::ECEFPoint routeBRoadSegmentsElementBoundingSphereCenter;
  ::ad::map::point::ECEFCoordinate routeBRoadSegmentsElementBoundingSphereCenterX(6400000);
  routeBRoadSegmentsElementBoundingSphereCenter.x = routeBRoadSegmentsElementBoundingSphereCenterX;
  ::ad::map::point::ECEFCoordinate routeBRoadSegmentsElementBoundingSphereCenterY(6400000);
  routeBRoadSegmentsElementBoundingSphereCenter.y = routeBRoadSegmentsElementBoundingSphereCenterY;
  ::ad::map::point::ECEFCoordinate routeBRoadSegmentsElementBoundingSphereCenterZ(6400000);
  routeBRoadSegmentsElementBoundingSphereCenter.z = routeBRoadSegmentsElementBoundingSphereCenterZ;
  routeBRoadSegmentsElementBoundingSphere.center = routeBRoadSegmentsElementBoundingSphereCenter;
  ::ad::physics::Distance routeBRoadSegmentsElementBoundingSphereRadius(1e9);
  routeBRoadSegmentsElementBoundingSphere.radius = routeBRoadSegmentsElementBoundingSphereRadius;
  routeBRoadSegmentsElement.boundingSphere = routeBRoadSegmentsElementBoundingSphere;
  routeBRoadSegments.resize(2, routeBRoadSegmentsElement);
  routeB.roadSegments = routeBRoadSegments;
  ::ad::map::route::RoutePlanningCounter routeBRoutePlanningCounter(
    std::numeric_limits<::ad::map::route::RoutePlanningCounter>::max());
  routeB.routePlanningCounter = routeBRoutePlanningCounter;
  ::ad::map::route::SegmentCounter routeBFullRouteSegmentCount(
    std::numeric_limits<::ad::map::route::SegmentCounter>::max());
  routeB.fullRouteSegmentCount = routeBFullRouteSegmentCount;
  ::ad::map::route::RouteLaneOffset routeBDestinationLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  routeB.destinationLaneOffset = routeBDestinationLaneOffset;
  ::ad::map::route::RouteLaneOffset routeBMinLaneOffset(std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  routeB.minLaneOffset = routeBMinLaneOffset;
  ::ad::map::route::RouteLaneOffset routeBMaxLaneOffset(std::numeric_limits<::ad::map::route::RouteLaneOffset>::max());
  routeB.maxLaneOffset = routeBMaxLaneOffset;
  ::ad::map::route::RouteCreationMode routeBRouteCreationMode(::ad::map::route::RouteCreationMode::AllNeighborLanes);
  routeB.routeCreationMode = routeBRouteCreationMode;
  valueA.routeB = routeB;
  ::ad::map::route::ConnectingRoute valueB = mValue;

  EXPECT_FALSE(valueA == valueB);
  EXPECT_TRUE(valueA != valueB);
}

#if defined(__clang__) && (__clang_major__ >= 7)
#pragma GCC diagnostic pop
#endif
