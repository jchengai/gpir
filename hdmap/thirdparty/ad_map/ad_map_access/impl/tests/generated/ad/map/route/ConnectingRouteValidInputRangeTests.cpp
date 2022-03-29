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

#include "ad/map/route/ConnectingRouteValidInputRange.hpp"

TEST(ConnectingRouteValidInputRangeTests, testValidInputRange)
{
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
  valueRouteARoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteARoadSegmentsElementDrivableLaneSegmentsElement);
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
  valueRouteBRoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement);
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
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(ConnectingRouteValidInputRangeTests, testValidInputRangeTypeTooSmall)
{
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
  valueRouteARoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteARoadSegmentsElementDrivableLaneSegmentsElement);
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
  valueRouteBRoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement);
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

  // override member with data type value below input range minimum
  ::ad::map::route::ConnectingRouteType invalidInitializedMember(
    static_cast<::ad::map::route::ConnectingRouteType>(-1));
  value.type = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ConnectingRouteValidInputRangeTests, testValidInputRangeTypeTooBig)
{
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
  valueRouteARoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteARoadSegmentsElementDrivableLaneSegmentsElement);
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
  valueRouteBRoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement);
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

  // override member with data type value above input range maximum
  ::ad::map::route::ConnectingRouteType invalidInitializedMember(
    static_cast<::ad::map::route::ConnectingRouteType>(-1));
  value.type = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ConnectingRouteValidInputRangeTests, testValidInputRangeRouteATooSmall)
{
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
  valueRouteARoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteARoadSegmentsElementDrivableLaneSegmentsElement);
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
  valueRouteBRoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement);
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

  // override member with data type value below input range minimum
  ::ad::map::route::FullRoute invalidInitializedMember;
  ::ad::map::route::RouteCreationMode invalidInitializedMemberRouteCreationMode(
    static_cast<::ad::map::route::RouteCreationMode>(-1));
  invalidInitializedMember.routeCreationMode = invalidInitializedMemberRouteCreationMode;
  value.routeA = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ConnectingRouteValidInputRangeTests, testValidInputRangeRouteATooBig)
{
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
  valueRouteARoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteARoadSegmentsElementDrivableLaneSegmentsElement);
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
  valueRouteBRoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement);
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

  // override member with data type value above input range maximum
  ::ad::map::route::FullRoute invalidInitializedMember;
  ::ad::map::route::RouteCreationMode invalidInitializedMemberRouteCreationMode(
    static_cast<::ad::map::route::RouteCreationMode>(-1));
  invalidInitializedMember.routeCreationMode = invalidInitializedMemberRouteCreationMode;
  value.routeA = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ConnectingRouteValidInputRangeTests, testValidInputRangeRouteBTooSmall)
{
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
  valueRouteARoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteARoadSegmentsElementDrivableLaneSegmentsElement);
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
  valueRouteBRoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement);
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

  // override member with data type value below input range minimum
  ::ad::map::route::FullRoute invalidInitializedMember;
  ::ad::map::route::RouteCreationMode invalidInitializedMemberRouteCreationMode(
    static_cast<::ad::map::route::RouteCreationMode>(-1));
  invalidInitializedMember.routeCreationMode = invalidInitializedMemberRouteCreationMode;
  value.routeB = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(ConnectingRouteValidInputRangeTests, testValidInputRangeRouteBTooBig)
{
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
  valueRouteARoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteARoadSegmentsElementDrivableLaneSegmentsElement);
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
  valueRouteBRoadSegmentsElementDrivableLaneSegments.resize(1,
                                                            valueRouteBRoadSegmentsElementDrivableLaneSegmentsElement);
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

  // override member with data type value above input range maximum
  ::ad::map::route::FullRoute invalidInitializedMember;
  ::ad::map::route::RouteCreationMode invalidInitializedMemberRouteCreationMode(
    static_cast<::ad::map::route::RouteCreationMode>(-1));
  invalidInitializedMember.routeCreationMode = invalidInitializedMemberRouteCreationMode;
  value.routeB = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
