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

#include "ad/map/route/RoadSegmentListValidInputRange.hpp"

TEST(RoadSegmentListValidInputRangeTests, testValidInputRangeValidInputRangeMin)
{
  ::ad::map::route::RoadSegmentList value;
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(RoadSegmentListValidInputRangeTests, testValidInputRangeElementValid)
{
  ::ad::map::route::RoadSegmentList value;
  ::ad::map::route::RoadSegment element;
  ::ad::map::route::LaneSegmentList elementDrivableLaneSegments;
  ::ad::map::route::LaneSegment elementDrivableLaneSegmentsElement;
  ::ad::map::lane::LaneId elementDrivableLaneSegmentsElementLeftNeighbor(1);
  elementDrivableLaneSegmentsElement.leftNeighbor = elementDrivableLaneSegmentsElementLeftNeighbor;
  ::ad::map::lane::LaneId elementDrivableLaneSegmentsElementRightNeighbor(1);
  elementDrivableLaneSegmentsElement.rightNeighbor = elementDrivableLaneSegmentsElementRightNeighbor;
  ::ad::map::lane::LaneIdList elementDrivableLaneSegmentsElementPredecessors;
  ::ad::map::lane::LaneId elementDrivableLaneSegmentsElementPredecessorsElement(1);
  elementDrivableLaneSegmentsElementPredecessors.resize(1, elementDrivableLaneSegmentsElementPredecessorsElement);
  elementDrivableLaneSegmentsElement.predecessors = elementDrivableLaneSegmentsElementPredecessors;
  ::ad::map::lane::LaneIdList elementDrivableLaneSegmentsElementSuccessors;
  ::ad::map::lane::LaneId elementDrivableLaneSegmentsElementSuccessorsElement(1);
  elementDrivableLaneSegmentsElementSuccessors.resize(1, elementDrivableLaneSegmentsElementSuccessorsElement);
  elementDrivableLaneSegmentsElement.successors = elementDrivableLaneSegmentsElementSuccessors;
  ::ad::map::route::LaneInterval elementDrivableLaneSegmentsElementLaneInterval;
  ::ad::map::lane::LaneId elementDrivableLaneSegmentsElementLaneIntervalLaneId(1);
  elementDrivableLaneSegmentsElementLaneInterval.laneId = elementDrivableLaneSegmentsElementLaneIntervalLaneId;
  ::ad::physics::ParametricValue elementDrivableLaneSegmentsElementLaneIntervalStart(0.);
  elementDrivableLaneSegmentsElementLaneInterval.start = elementDrivableLaneSegmentsElementLaneIntervalStart;
  ::ad::physics::ParametricValue elementDrivableLaneSegmentsElementLaneIntervalEnd(0.);
  elementDrivableLaneSegmentsElementLaneInterval.end = elementDrivableLaneSegmentsElementLaneIntervalEnd;
  bool elementDrivableLaneSegmentsElementLaneIntervalWrongWay{true};
  elementDrivableLaneSegmentsElementLaneInterval.wrongWay = elementDrivableLaneSegmentsElementLaneIntervalWrongWay;
  elementDrivableLaneSegmentsElement.laneInterval = elementDrivableLaneSegmentsElementLaneInterval;
  ::ad::map::route::RouteLaneOffset elementDrivableLaneSegmentsElementRouteLaneOffset(
    std::numeric_limits<::ad::map::route::RouteLaneOffset>::lowest());
  elementDrivableLaneSegmentsElement.routeLaneOffset = elementDrivableLaneSegmentsElementRouteLaneOffset;
  elementDrivableLaneSegments.resize(1, elementDrivableLaneSegmentsElement);
  element.drivableLaneSegments = elementDrivableLaneSegments;
  ::ad::map::route::SegmentCounter elementSegmentCountFromDestination(
    std::numeric_limits<::ad::map::route::SegmentCounter>::lowest());
  element.segmentCountFromDestination = elementSegmentCountFromDestination;
  ::ad::map::point::BoundingSphere elementBoundingSphere;
  ::ad::map::point::ECEFPoint elementBoundingSphereCenter;
  ::ad::map::point::ECEFCoordinate elementBoundingSphereCenterX(-6400000);
  elementBoundingSphereCenter.x = elementBoundingSphereCenterX;
  ::ad::map::point::ECEFCoordinate elementBoundingSphereCenterY(-6400000);
  elementBoundingSphereCenter.y = elementBoundingSphereCenterY;
  ::ad::map::point::ECEFCoordinate elementBoundingSphereCenterZ(-6400000);
  elementBoundingSphereCenter.z = elementBoundingSphereCenterZ;
  elementBoundingSphere.center = elementBoundingSphereCenter;
  ::ad::physics::Distance elementBoundingSphereRadius(-1e9);
  elementBoundingSphere.radius = elementBoundingSphereRadius;
  element.boundingSphere = elementBoundingSphere;
  value.push_back(element);
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(RoadSegmentListValidInputRangeTests, testValidInputRangeElementInvalid)
{
  ::ad::map::route::RoadSegmentList value;
  ::ad::map::route::RoadSegment element;
  ::ad::map::point::BoundingSphere elementBoundingSphere;
  ::ad::map::point::ECEFPoint elementBoundingSphereCenter;
  ::ad::map::point::ECEFCoordinate elementBoundingSphereCenterX(-6400000 * 1.1);
  elementBoundingSphereCenter.x = elementBoundingSphereCenterX;
  elementBoundingSphere.center = elementBoundingSphereCenter;
  element.boundingSphere = elementBoundingSphere;
  value.push_back(element);
  ASSERT_FALSE(withinValidInputRange(value));
}
