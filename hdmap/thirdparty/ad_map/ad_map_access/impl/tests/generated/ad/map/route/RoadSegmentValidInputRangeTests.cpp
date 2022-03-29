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

#include "ad/map/route/RoadSegmentValidInputRange.hpp"

TEST(RoadSegmentValidInputRangeTests, testValidInputRange)
{
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
  ASSERT_TRUE(withinValidInputRange(value));
}

TEST(RoadSegmentValidInputRangeTests, testValidInputRangeBoundingSphereTooSmall)
{
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

  // override member with data type value below input range minimum
  ::ad::map::point::BoundingSphere invalidInitializedMember;
  ::ad::map::point::ECEFPoint invalidInitializedMemberCenter;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberCenterX(-6400000 * 1.1);
  invalidInitializedMemberCenter.x = invalidInitializedMemberCenterX;
  invalidInitializedMember.center = invalidInitializedMemberCenter;
  value.boundingSphere = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}

TEST(RoadSegmentValidInputRangeTests, testValidInputRangeBoundingSphereTooBig)
{
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

  // override member with data type value above input range maximum
  ::ad::map::point::BoundingSphere invalidInitializedMember;
  ::ad::map::point::ECEFPoint invalidInitializedMemberCenter;
  ::ad::map::point::ECEFCoordinate invalidInitializedMemberCenterX(6400000 * 1.1);
  invalidInitializedMemberCenter.x = invalidInitializedMemberCenterX;
  invalidInitializedMember.center = invalidInitializedMemberCenter;
  value.boundingSphere = invalidInitializedMember;
  ASSERT_FALSE(withinValidInputRange(value));
}
