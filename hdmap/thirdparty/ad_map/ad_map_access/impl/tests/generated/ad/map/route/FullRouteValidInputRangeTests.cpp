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

#include "ad/map/route/FullRouteValidInputRange.hpp"

TEST(FullRouteValidInputRangeTests, testValidInputRange)
{
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
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(FullRouteValidInputRangeTests, testValidInputRangeRouteCreationModeTooSmall)
{
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

  // override member with data type value below input range minimum
  ::ad::map::route::RouteCreationMode invalidInitializedMember(static_cast<::ad::map::route::RouteCreationMode>(-1));
  value.routeCreationMode = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(FullRouteValidInputRangeTests, testValidInputRangeRouteCreationModeTooBig)
{
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

  // override member with data type value above input range maximum
  ::ad::map::route::RouteCreationMode invalidInitializedMember(static_cast<::ad::map::route::RouteCreationMode>(-1));
  value.routeCreationMode = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
