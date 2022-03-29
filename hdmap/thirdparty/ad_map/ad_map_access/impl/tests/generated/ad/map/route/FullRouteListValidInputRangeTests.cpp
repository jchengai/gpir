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

#include "ad/map/route/FullRouteListValidInputRange.hpp"

TEST(FullRouteListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::route::FullRouteList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(FullRouteListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::route::FullRouteList value;
  ::ad::map::route::FullRoute element;
  ::ad::map::route::RoadSegmentList elementRoadSegments;
  ::ad::map::route::RoadSegment elementRoadSegmentsElement;
  ::ad::map::route::LaneSegmentList elementRoadSegmentsElementDrivableLaneSegments;
  ::ad::map::route::LaneSegment elementRoadSegmentsElementDrivableLaneSegmentsElement;
  ::ad::map::lane::LaneId elementRoadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor(1);
  elementRoadSegmentsElementDrivableLaneSegmentsElement.leftNeighbor
    = elementRoadSegmentsElementDrivableLaneSegmentsElementLeftNeighbor;
  ::ad::map::lane::LaneId elementRoadSegmentsElementDrivableLaneSegmentsElementRightNeighbor(1);
  elementRoadSegmentsElementDrivableLaneSegmentsElement.rightNeighbor
    = elementRoadSegmentsElementDrivableLaneSegmentsElementRightNeighbor;
  ::ad::map::lane::LaneIdList elementRoadSegmentsElementDrivableLaneSegmentsElementPredecessors;
  ::ad::map::lane::LaneId elementRoadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement(1);
  elementRoadSegmentsElementDrivableLaneSegmentsElementPredecessors.resize(
    1, elementRoadSegmentsElementDrivableLaneSegmentsElementPredecessorsElement);
  elementRoadSegmentsElementDrivableLaneSegmentsElement.predecessors
    = elementRoadSegmentsElementDrivableLaneSegmentsElementPredecessors;
  ::ad::map::lane::LaneIdList elementRoadSegmentsElementDrivableLaneSegmentsElementSuccessors;
  ::ad::map::lane::LaneId elementRoadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement(1);
  elementRoadSegmentsElementDrivableLaneSegmentsElementSuccessors.resize(
    1, elementRoadSegmentsElementDrivableLaneSegmentsElementSuccessorsElement);
  elementRoadSegmentsElementDrivableLaneSegmentsElement.successors
    = elementRoadSegmentsElementDrivableLaneSegmentsElementSuccessors;
  ::ad::map::route::LaneInterval elementRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
  ::ad::map::lane::LaneId elementRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId(1);
  elementRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.laneId
    = elementRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalLaneId;
  ::ad::physics::ParametricValue elementRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart(0.);
  elementRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.start
    = elementRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalStart;
  ::ad::physics::ParametricValue elementRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd(0.);
  elementRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.end
    = elementRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalEnd;
  bool elementRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay{true};
  elementRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval.wrongWay
    = elementRoadSegmentsElementDrivableLaneSegmentsElementLaneIntervalWrongWay;
  elementRoadSegmentsElementDrivableLaneSegmentsElement.laneInterval
    = elementRoadSegmentsElementDrivableLaneSegmentsElementLaneInterval;
  ::ad::map::route::RouteLaneOffset elementRoadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
  elementRoadSegmentsElementDrivableLaneSegmentsElement.routeLaneOffset
    = elementRoadSegmentsElementDrivableLaneSegmentsElementRouteLaneOffset;
  elementRoadSegmentsElementDrivableLaneSegments.resize(1, elementRoadSegmentsElementDrivableLaneSegmentsElement);
  elementRoadSegmentsElement.drivableLaneSegments = elementRoadSegmentsElementDrivableLaneSegments;
  ::ad::map::route::SegmentCounter elementRoadSegmentsElementSegmentCountFromDestination(
    std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
  elementRoadSegmentsElement.segmentCountFromDestination = elementRoadSegmentsElementSegmentCountFromDestination;
  ::ad::map::point::BoundingSphere elementRoadSegmentsElementBoundingSphere;
  ::ad::map::point::ECEFPoint elementRoadSegmentsElementBoundingSphereCenter;
  ::ad::map::point::ECEFCoordinate elementRoadSegmentsElementBoundingSphereCenterX(-6400000);
  elementRoadSegmentsElementBoundingSphereCenter.x = elementRoadSegmentsElementBoundingSphereCenterX;
  ::ad::map::point::ECEFCoordinate elementRoadSegmentsElementBoundingSphereCenterY(-6400000);
  elementRoadSegmentsElementBoundingSphereCenter.y = elementRoadSegmentsElementBoundingSphereCenterY;
  ::ad::map::point::ECEFCoordinate elementRoadSegmentsElementBoundingSphereCenterZ(-6400000);
  elementRoadSegmentsElementBoundingSphereCenter.z = elementRoadSegmentsElementBoundingSphereCenterZ;
  elementRoadSegmentsElementBoundingSphere.center = elementRoadSegmentsElementBoundingSphereCenter;
  ::ad::physics::Distance elementRoadSegmentsElementBoundingSphereRadius(-1e9);
  elementRoadSegmentsElementBoundingSphere.radius = elementRoadSegmentsElementBoundingSphereRadius;
  elementRoadSegmentsElement.boundingSphere = elementRoadSegmentsElementBoundingSphere;
  elementRoadSegments.resize(1, elementRoadSegmentsElement);
  element.roadSegments = elementRoadSegments;
  ::ad::map::route::RoutePlanningCounter elementRoutePlanningCounter(
    std::numeric_limits<::ad::map::route::RoutePlanningCounter>::lowest());
  element.routePlanningCounter = elementRoutePlanningCounter;
  ::ad::map::route::SegmentCounter elementFullRouteSegmentCount(
    std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
  element.fullRouteSegmentCount = elementFullRouteSegmentCount;
  ::ad::map::route::RouteLaneOffset elementDestinationLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
  element.destinationLaneOffset = elementDestinationLaneOffset;
  ::ad::map::route::RouteLaneOffset elementMinLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
  element.minLaneOffset = elementMinLaneOffset;
  ::ad::map::route::RouteLaneOffset elementMaxLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
  element.maxLaneOffset = elementMaxLaneOffset;
  ::ad::map::route::RouteCreationMode elementRouteCreationMode(::ad::map::route::RouteCreationMode::Undefined);
  element.routeCreationMode = elementRouteCreationMode;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(FullRouteListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::route::FullRouteList value;
  ::ad::map::route::FullRoute element;
  ::ad::map::route::RouteCreationMode elementRouteCreationMode(static_cast<::ad::map::route::RouteCreationMode>(-1));
  element.routeCreationMode = elementRouteCreationMode;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
