// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/access/Operation.hpp>
#include <ad/map/lane/Lane.hpp>
#include <ad/map/lane/LaneOperation.hpp>
#include <ad/map/match/AdMapMatching.hpp>
#include <ad/map/match/Types.hpp>
#include <ad/map/point/Operation.hpp>
#include <ad/map/route/Planning.hpp>
#include <ad/map/route/RouteOperation.hpp>
#include <gtest/gtest.h>

using namespace ::ad;
using namespace map;
using namespace map::route;
using namespace map::route::planning;
using namespace map::point;

struct RoutePlanningTest : ::testing::Test
{
  RoutePlanningTest()
  {
  }

  virtual ~RoutePlanningTest() = default;

  virtual void SetUp()
  {
    access::cleanup();
  }

  virtual void TearDown()
  {
    access::cleanup();
  }

  route::FullRoute prepareTpkRoute()
  {
    if (!access::init(mTestFile))
    {
      throw std::runtime_error("Unable to initialize with " + mTestFile);
    }

    /* 437 */ startLaneId
      = lane::uniqueLaneId(createGeoPoint(Longitude(8.4404755), Latitude(49.0195732), Altitude(0.))); // id: 2682
    /* 314 */ firstLaneId
      = lane::uniqueLaneId(createGeoPoint(Longitude(8.4406803),
                                          Latitude(49.0197914),
                                          Altitude(0.))); // lane inside intersection, todo more description, id: 2462
    /* 402 */ secondLaneId
      = lane::uniqueLaneId(createGeoPoint(Longitude(8.4407235),
                                          Latitude(49.0197907),
                                          Altitude(0.))); // lane after intersection, todo more description, id: 3018
    /* 138 */ thirdLaneId
      = lane::uniqueLaneId(createGeoPoint(Longitude(8.4418464),
                                          Latitude(49.0193326),
                                          Altitude(0.))); // lane inside intersection, todo more description, id: 2227
    /* 247 */ fourthLaneId
      = lane::uniqueLaneId(createGeoPoint(Longitude(8.4421272),
                                          Latitude(49.0192331),
                                          Altitude(0.))); // lane inside intersection, todo more description, id: 1093
    /* 408 */ fifthLaneId
      = lane::uniqueLaneId(createGeoPoint(Longitude(8.4421012),
                                          Latitude(49.0192655),
                                          Altitude(0.))); // lane inside intersection, todo more description, id: 1071
    /* 415 */ endLaneId = lane::uniqueLaneId(createGeoPoint(Longitude(8.4422283), Latitude(49.0192052), Altitude(0.)));

    return route::planning::planRoute(createParaPoint(startLaneId, physics::ParametricValue(0.2)),
                                      createParaPoint(endLaneId, physics::ParametricValue(0.7)));
  }

  void eraseFirstSegment(route::FullRoute &route);
  enum AlignmentBehavior
  {
    Equal,
    EndSegmentDiffers,
  };
  void compareRoutes(route::FullRoute const &left,
                     route::FullRoute const &right,
                     AlignmentBehavior const alignmentBehavior = Equal);
  enum RouteBehaviour
  {
    RouteIsConstant,
    RouteIsBroadening,
    RouteIsNarrowing,
  };
  void validateRouteConnections(route::FullRoute const &route, RouteBehaviour const routeBehavior = RouteIsConstant);
  void validateRouteParaPoints(route::FullRoute const &route);

  std::string mTestFile{"test_files/TPK.adm.txt"};

  std::vector<std::pair<GeoPoint, size_t>> mTestPoints{
    std::make_pair(createGeoPoint(Longitude(8.4400665), Latitude(49.0192005), Altitude(0.)), 0u),
    std::make_pair(createGeoPoint(Longitude(8.4401882), Latitude(49.0191939), Altitude(0.)), 1u),
    std::make_pair(createGeoPoint(Longitude(8.4401510), Latitude(49.0191792), Altitude(0.)), 2u),
    std::make_pair(createGeoPoint(Longitude(8.4401742), Latitude(49.0192009), Altitude(0.)), 3u),
    // approaching middle of the intersection
    std::make_pair(createGeoPoint(Longitude(8.4401540), Latitude(49.0192082), Altitude(0.)), 4u)};
  lane::LaneId startLaneId{};
  lane::LaneId firstLaneId{};
  lane::LaneId secondLaneId{};
  lane::LaneId thirdLaneId{};
  lane::LaneId fourthLaneId{};
  lane::LaneId fifthLaneId{};
  lane::LaneId endLaneId{};
};

void RoutePlanningTest::eraseFirstSegment(route::FullRoute &route)
{
  route.roadSegments.erase(route.roadSegments.begin());
  for (auto &laneSegment : route.roadSegments.front().drivableLaneSegments)
  {
    laneSegment.predecessors.clear();
  }
}

void RoutePlanningTest::compareRoutes(route::FullRoute const &left,
                                      route::FullRoute const &right,
                                      AlignmentBehavior const alignmentBehavior)
{
  EXPECT_EQ(left.roadSegments.size(), right.roadSegments.size());
  for (std::size_t i = 0u; i < left.roadSegments.size(); ++i)
  {
    EXPECT_EQ(left.roadSegments[i].boundingSphere, right.roadSegments[i].boundingSphere);
    EXPECT_EQ(left.roadSegments[i].segmentCountFromDestination, right.roadSegments[i].segmentCountFromDestination);
    EXPECT_EQ(left.roadSegments[i].drivableLaneSegments.size(), right.roadSegments[i].drivableLaneSegments.size());
    for (std::size_t j = 0u; j < left.roadSegments[i].drivableLaneSegments.size(); ++j)
    {
      EXPECT_EQ(left.roadSegments[i].drivableLaneSegments[j].laneInterval.laneId,
                right.roadSegments[i].drivableLaneSegments[j].laneInterval.laneId);
      EXPECT_EQ(left.roadSegments[i].drivableLaneSegments[j].laneInterval.wrongWay,
                right.roadSegments[i].drivableLaneSegments[j].laneInterval.wrongWay);
      EXPECT_EQ(left.roadSegments[i].drivableLaneSegments[j].laneInterval.start,
                right.roadSegments[i].drivableLaneSegments[j].laneInterval.start);

      if ((alignmentBehavior == EndSegmentDiffers) && (i + 1 == left.roadSegments.size()))
      {
        EXPECT_NE(left.roadSegments[i].drivableLaneSegments[j].laneInterval.end,
                  right.roadSegments[i].drivableLaneSegments[j].laneInterval.end);
        EXPECT_LE(std::fabs(left.roadSegments[i].drivableLaneSegments[j].laneInterval.end
                            - right.roadSegments[i].drivableLaneSegments[j].laneInterval.end),
                  physics::ParametricValue(0.1));
      }
      else
      {
        EXPECT_EQ(left.roadSegments[i].drivableLaneSegments[j].laneInterval.end,
                  right.roadSegments[i].drivableLaneSegments[j].laneInterval.end);
      }
    }
  }
  EXPECT_EQ(left.fullRouteSegmentCount, right.fullRouteSegmentCount);
  validateRouteParaPoints(left);
}

void RoutePlanningTest::validateRouteParaPoints(route::FullRoute const &route)
{
  for (std::size_t i = 0u; i < route.roadSegments.size(); ++i)
  {
    for (std::size_t j = 0u; j < route.roadSegments[i].drivableLaneSegments.size(); ++j)
    {
      // TParam is always 0 or 1; except for the start and the end of the route
      if ((i != 0u) && (i != route.roadSegments.size() - 1u))
      {
        EXPECT_TRUE(
          (physics::ParametricValue(0.) == route.roadSegments[i].drivableLaneSegments[j].laneInterval.start)
          || (physics::ParametricValue(1.) == route.roadSegments[i].drivableLaneSegments[j].laneInterval.start));
        EXPECT_TRUE(
          (physics::ParametricValue(0.) == route.roadSegments[i].drivableLaneSegments[j].laneInterval.end)
          || (physics::ParametricValue(1.) == route.roadSegments[i].drivableLaneSegments[j].laneInterval.end));
      }
      EXPECT_NE(isRouteDirectionAlignedWithDrivingDirection(route.roadSegments[i].drivableLaneSegments[j].laneInterval),
                route.roadSegments[i].drivableLaneSegments[j].laneInterval.wrongWay)
        << route.roadSegments[i].drivableLaneSegments[j].laneInterval;
    }
  }
}

void RoutePlanningTest::validateRouteConnections(route::FullRoute const &route, RouteBehaviour const routeBehavior)
{
  std::vector<std::set<lane::LaneId>> actualPresentLaneIds;
  std::vector<std::set<lane::LaneId>> listedPredecessors;
  std::vector<std::set<lane::LaneId>> listedSuccessors;
  actualPresentLaneIds.resize(route.roadSegments.size());
  listedPredecessors.resize(route.roadSegments.size());
  listedSuccessors.resize(route.roadSegments.size());
  bool maxLaneOffsetPresent = route.roadSegments.empty();
  bool minLaneOffsetPresent = route.roadSegments.empty();
  for (std::size_t i = 0u; i < route.roadSegments.size(); ++i)
  {
    if (route.roadSegments[i].drivableLaneSegments[0].routeLaneOffset < 0)
    {
      EXPECT_EQ(RouteIsBroadening, routeBehavior);
    }
    else if (route.roadSegments[i].drivableLaneSegments[0].routeLaneOffset > 0)
    {
      EXPECT_EQ(RouteIsNarrowing, routeBehavior);
    }
    for (std::size_t j = 0u; j < route.roadSegments[i].drivableLaneSegments.size(); ++j)
    {
      EXPECT_LE(route.roadSegments[i].drivableLaneSegments[j].routeLaneOffset, route.maxLaneOffset)
        << "i:" << i << " j:" << j << " route:" << route;
      EXPECT_GE(route.roadSegments[i].drivableLaneSegments[j].routeLaneOffset, route.minLaneOffset)
        << "i:" << i << " j:" << j << " route:" << route;
      maxLaneOffsetPresent |= route.roadSegments[i].drivableLaneSegments[j].routeLaneOffset == route.maxLaneOffset;
      minLaneOffsetPresent |= route.roadSegments[i].drivableLaneSegments[j].routeLaneOffset == route.minLaneOffset;

      EXPECT_EQ(int(j),
                route.roadSegments[i].drivableLaneSegments[j].routeLaneOffset
                  - route.roadSegments[i].drivableLaneSegments[0].routeLaneOffset)
        << "i:" << i << " j:" << j << " route:" << route;

      if (static_cast<int32_t>(j) != route.roadSegments[i].drivableLaneSegments[j].routeLaneOffset)
      {
        EXPECT_NE(RouteIsConstant, routeBehavior);
      }

      actualPresentLaneIds[i].insert(route.roadSegments[i].drivableLaneSegments[j].laneInterval.laneId);
      listedPredecessors[i].insert(std::begin(route.roadSegments[i].drivableLaneSegments[j].predecessors),
                                   std::end(route.roadSegments[i].drivableLaneSegments[j].predecessors));
      listedSuccessors[i].insert(std::begin(route.roadSegments[i].drivableLaneSegments[j].successors),
                                 std::end(route.roadSegments[i].drivableLaneSegments[j].successors));
    }
  }
  EXPECT_TRUE(maxLaneOffsetPresent);
  EXPECT_TRUE(minLaneOffsetPresent);

  for (std::size_t i = 0u; i < route.roadSegments.size(); ++i)
  {
    for (std::size_t j = 0u; j < route.roadSegments[i].drivableLaneSegments.size(); ++j)
    {
      for (auto predecessorId : route.roadSegments[i].drivableLaneSegments[j].predecessors)
      {
        // no predecessors in first road segment allowed
        ASSERT_NE(i, 0u);
        ASSERT_TRUE(actualPresentLaneIds[i - 1].find(predecessorId) != actualPresentLaneIds[i - 1].end());
      }
      for (auto successorId : route.roadSegments[i].drivableLaneSegments[j].successors)
      {
        // no successors in last road segment allowed
        ASSERT_NE(i, route.roadSegments.size() - 1u);
        ASSERT_TRUE(actualPresentLaneIds[i + 1].find(successorId) != actualPresentLaneIds[i + 1].end());
      }
      if (j == 0u)
      {
        ASSERT_EQ(route.roadSegments[i].drivableLaneSegments[j].rightNeighbor, lane::LaneId());
      }
      else
      {
        ASSERT_TRUE(actualPresentLaneIds[i].find(route.roadSegments[i].drivableLaneSegments[j].rightNeighbor)
                    != actualPresentLaneIds[i].end());
      }
      if (j + 1 == route.roadSegments[i].drivableLaneSegments.size())
      {
        ASSERT_EQ(route.roadSegments[i].drivableLaneSegments[j].leftNeighbor, lane::LaneId());
      }
      else
      {
        ASSERT_TRUE(actualPresentLaneIds[i].find(route.roadSegments[i].drivableLaneSegments[j].leftNeighbor)
                    != actualPresentLaneIds[i].end());
      }
    }
    if (i > 0u)
    {
      // all previous lanes are listed as predecessor in this segment
      if (actualPresentLaneIds[i - 1].size() > listedPredecessors[i].size())
      {
        // there are additional lanes in the previous segment,
        ASSERT_EQ(RouteIsNarrowing, routeBehavior);
        // ensure that at least all listed predecessors exist
        ASSERT_GE(listedPredecessors[i].size(), 1u);
        for (auto listedPredecessor : listedPredecessors[i])
        {
          auto findResult = actualPresentLaneIds[i - 1].find(listedPredecessor);
          ASSERT_TRUE(findResult != actualPresentLaneIds[i - 1].end());
        }
      }
      else
      {
        ASSERT_EQ(actualPresentLaneIds[i - 1].size(), listedPredecessors[i].size());
      }
    }
    if (i < route.roadSegments.size() - 1u)
    {
      // all next lanes are listed as successors in this segment
      if (actualPresentLaneIds[i + 1].size() > listedSuccessors[i].size())
      {
        // there are additional lanes in the successor segment,
        ASSERT_EQ(RouteIsBroadening, routeBehavior);
        // ensure that at least all listed successors exist
        ASSERT_GE(listedSuccessors[i].size(), 1u);
        for (auto listedSuccessor : listedSuccessors[i])
        {
          auto findResult = actualPresentLaneIds[i + 1].find(listedSuccessor);
          ASSERT_TRUE(findResult != actualPresentLaneIds[i + 1].end());
        }
      }
      else
      {
        ASSERT_EQ(actualPresentLaneIds[i + 1].size(), listedSuccessors[i].size());
      }
    }
  }
}

TEST_F(RoutePlanningTest, route_planning)
{
  using namespace route;
  using namespace route::planning;

  struct RouteValues
  {
    size_t routeSize;
    double routeLength;
  };

  ASSERT_TRUE(access::init(mTestFile));
  access::setENUReferencePoint(mTestPoints.back().first);

  startLaneId = lane::uniqueLaneId(createGeoPoint(Longitude(8.4404755), Latitude(49.0195732), Altitude(0.)));
  endLaneId = lane::uniqueLaneId(createGeoPoint(Longitude(8.4422283), Latitude(49.0192052), Altitude(0.)));
  auto startLaneDrivingDirectionNegative
    = lane::uniqueLaneId(createGeoPoint(Longitude(8.43988334), Latitude(49.02012825), Altitude(0.)));
  std::vector<std::pair<std::pair<RoutingParaPoint, RoutingParaPoint>, RouteValues>> routesToPlan{
    std::make_pair(std::make_pair(createRoutingPoint(startLaneId, physics::ParametricValue(0.2)),
                                  createRoutingPoint(startLaneId, physics::ParametricValue(0.7))),
                   RouteValues{1u, 35.2}),
    // positive direction should give same results
    std::make_pair(
      std::make_pair(createRoutingPoint(startLaneId, physics::ParametricValue(0.2), RoutingDirection::POSITIVE),
                     createRoutingPoint(startLaneId, physics::ParametricValue(0.7), RoutingDirection::POSITIVE)),
      RouteValues{1u, 35.2}),
    // ENUHeading of zero should be positive direction (lane is oriented towards north/east
    std::make_pair(
      std::make_pair(createRoutingPoint(createParaPoint(startLaneId, physics::ParametricValue(0.2)), ENUHeading(0.)),
                     createRoutingPoint(startLaneId, physics::ParametricValue(0.7))),
      RouteValues{1u, 35.2}),
    // negative direction should fail
    std::make_pair(
      std::make_pair(createRoutingPoint(startLaneId, physics::ParametricValue(0.2), RoutingDirection::NEGATIVE),
                     createRoutingPoint(startLaneId, physics::ParametricValue(0.7))),
      RouteValues{0u, 0.}),
    // ENUHeading of 3.5 should be negative direction (lane is oriented towards north/east
    std::make_pair(
      std::make_pair(createRoutingPoint(createParaPoint(startLaneId, physics::ParametricValue(0.2)), ENUHeading(3.5)),
                     createRoutingPoint(startLaneId, physics::ParametricValue(0.7))),
      RouteValues{0u, 0.}),
    // using negative driving direction lane
    std::make_pair(std::make_pair(createRoutingPoint(startLaneDrivingDirectionNegative, physics::ParametricValue(0.7)),
                                  createRoutingPoint(startLaneDrivingDirectionNegative, physics::ParametricValue(0.2))),
                   RouteValues{1u, 95.75}),
    std::make_pair(std::make_pair(createRoutingPoint(startLaneDrivingDirectionNegative,
                                                     physics::ParametricValue(0.7),
                                                     RoutingDirection::NEGATIVE),
                                  createRoutingPoint(startLaneDrivingDirectionNegative, physics::ParametricValue(0.2))),
                   RouteValues{1u, 95.75}),
    // ENUHeading of 0. should be positive lane direction and so negative routing direction (lane is orientated towards
    // south/east)
    std::make_pair(std::make_pair(createRoutingPoint(
                                    createParaPoint(startLaneDrivingDirectionNegative, physics::ParametricValue(0.7)),
                                    ENUHeading(0.)),
                                  createRoutingPoint(startLaneDrivingDirectionNegative, physics::ParametricValue(0.2))),
                   RouteValues{1u, 95.75}),
    std::make_pair(std::make_pair(createRoutingPoint(startLaneDrivingDirectionNegative,
                                                     physics::ParametricValue(0.7),
                                                     RoutingDirection::POSITIVE),
                                  createRoutingPoint(startLaneDrivingDirectionNegative, physics::ParametricValue(0.2))),
                   RouteValues{0u, 0.}),
    std::make_pair(std::make_pair(createRoutingPoint(
                                    createParaPoint(startLaneDrivingDirectionNegative, physics::ParametricValue(0.7)),
                                    ENUHeading(3.5)),
                                  createRoutingPoint(startLaneDrivingDirectionNegative, physics::ParametricValue(0.2))),
                   RouteValues{0u, 0.}),
    std::make_pair(std::make_pair(createRoutingPoint(startLaneId, physics::ParametricValue(0.2)),
                                  createRoutingPoint(endLaneId, physics::ParametricValue(0.7))),
                   RouteValues{7u, 195.31}),
    std::make_pair(
      std::make_pair(createRoutingPoint(startLaneId, physics::ParametricValue(0.2), RoutingDirection::POSITIVE),
                     createRoutingPoint(endLaneId, physics::ParametricValue(0.7))),
      RouteValues{7u, 195.31}),
    std::make_pair(
      std::make_pair(createRoutingPoint(createParaPoint(startLaneId, physics::ParametricValue(0.2)), ENUHeading(0.)),
                     createRoutingPoint(endLaneId, physics::ParametricValue(0.7))),
      RouteValues{7u, 195.31}),
    std::make_pair(std::make_pair(createRoutingPoint(startLaneId, physics::ParametricValue(0.7)),
                                  createRoutingPoint(startLaneId, physics::ParametricValue(0.2))),
                   RouteValues{11u, 614.45}),
  };

  for (auto routeToPlan : routesToPlan)
  {
    auto routeResult = route::planning::planRoute(routeToPlan.first.first, routeToPlan.first.second);

    auto routeLength = route::calcLength(routeResult);

    ASSERT_EQ(routeToPlan.second.routeSize, routeResult.roadSegments.size()) << mTestFile;
    ASSERT_NEAR(double(routeToPlan.second.routeLength), double(routeLength), 1.) << mTestFile;

    // validate route connections
    validateRouteConnections(routeResult);
  }
}

TEST_F(RoutePlanningTest, empty_route_following)
{
  route::FullRoute emptyRoute;

  auto const paraPoint = createParaPoint(lane::LaneId(437), physics::ParametricValue(0.1));

  route::FindWaypointResult invalidResult(emptyRoute);
  ASSERT_FALSE(invalidResult.isValid());

  auto findWaypointResult = route::findWaypoint(paraPoint, emptyRoute);
  ASSERT_FALSE(findWaypointResult.isValid());
}

TEST_F(RoutePlanningTest, empty_mapmatched_bounding_box)
{
  route::FullRoute emptyRoute;
  match::MapMatchedObjectBoundingBox emptyObjectBoundingBox;
  auto findWaypointResult = route::objectOnRoute(emptyObjectBoundingBox, emptyRoute);
  ASSERT_FALSE(findWaypointResult.isValid());
}

TEST_F(RoutePlanningTest, route_following)
{
  route::FullRoute routeResult = prepareTpkRoute();

  route::FullRoute route = routeResult;
  validateRouteConnections(route);
  route::FullRoute compareRoute = routeResult;

  // position before the route: keep as is
  ASSERT_EQ(route::ShortenRouteResult::SucceededBeforeRoute,
            route::shortenRoute(createParaPoint(startLaneId, physics::ParametricValue(0.1)), route));
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  ASSERT_GT(compareRoute.roadSegments.size(), 0u);
  // position within the first segment: shorten first segment
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(startLaneId, physics::ParametricValue(0.4)), route));
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(0.4);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // position exactly at the end of the segment: first segment will degenerate, so drop it
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(startLaneId, physics::ParametricValue(1.)), route));
  eraseFirstSegment(compareRoute);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // actually entering next segment
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(firstLaneId, physics::ParametricValue(.01)), route));
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.01);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // travel in the middle of the segment
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(firstLaneId, physics::ParametricValue(.5)), route));
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.5);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // store current state before entering the next segment (which has negative driving direction!)
  routeResult = compareRoute;

  // jump into at the entering point
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(secondLaneId, physics::ParametricValue(1.)), route));
  eraseFirstSegment(compareRoute);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // jump into at the middle point
  route = routeResult;
  compareRoute = routeResult;
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(secondLaneId, physics::ParametricValue(.5)), route));
  eraseFirstSegment(compareRoute);
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.5);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // jitter a bit back: no change
  ASSERT_EQ(route::ShortenRouteResult::SucceededBeforeRoute,
            route::shortenRoute(createParaPoint(secondLaneId, physics::ParametricValue(0.51)), route));
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // jitter further back: no change
  ASSERT_EQ(route::ShortenRouteResult::SucceededBeforeRoute,
            route::shortenRoute(createParaPoint(secondLaneId, physics::ParametricValue(0.6)), route));
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // jitter a bit forward:
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(secondLaneId, physics::ParametricValue(0.49)), route));
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.49);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // jitter further forward:
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(secondLaneId, physics::ParametricValue(0.4)), route));
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.4);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // jump into the end point, old 3 segment points are removed
  route = routeResult;
  compareRoute = routeResult;
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(secondLaneId, physics::ParametricValue(0.)), route));
  eraseFirstSegment(compareRoute);
  eraseFirstSegment(compareRoute);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // same real point, but start of the next lane
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(thirdLaneId, physics::ParametricValue(0.)), route));
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // normal forward
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(thirdLaneId, physics::ParametricValue(0.5)), route));
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.5);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // jitter a bit back: no change
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(thirdLaneId, physics::ParametricValue(0.49)), route));
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // jitter further back: no change, but actually before route
  ASSERT_EQ(route::ShortenRouteResult::SucceededBeforeRoute,
            route::shortenRoute(createParaPoint(thirdLaneId, physics::ParametricValue(0.4)), route));
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // jitter the same, but with PrependIfSucceededBeforeRoute shorten route mode, should adapt the route
  // and expand to the front again
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(thirdLaneId, physics::ParametricValue(0.4)),
                                route,
                                route::ShortenRouteMode::PrependIfSucceededBeforeRoute));
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.4);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // normal forward to .5 again
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(thirdLaneId, physics::ParametricValue(0.5)), route));
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.5);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // jitter a bit forward
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(thirdLaneId, physics::ParametricValue(0.51)), route));
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.51);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // jitter further forward
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(thirdLaneId, physics::ParametricValue(0.6)), route));
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.6);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // jump over one segment completely
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(fourthLaneId, physics::ParametricValue(0.5)), route));
  eraseFirstSegment(compareRoute);
  eraseFirstSegment(compareRoute);
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.5);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // drive into the end segment
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(endLaneId, physics::ParametricValue(0.2)), route));
  eraseFirstSegment(compareRoute);
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.2);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // finishing nearby
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(endLaneId, physics::ParametricValue(0.69)), route));
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.69);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // finishing exactly
  ASSERT_EQ(route::ShortenRouteResult::SucceededRouteEmpty,
            route::shortenRoute(createParaPoint(endLaneId, physics::ParametricValue(0.7)), route));
  eraseFirstSegment(compareRoute);
  compareRoutes(compareRoute, route);
  ASSERT_TRUE(route.roadSegments.empty());

  // finishing directly after target
  route = routeResult;
  ASSERT_EQ(route::ShortenRouteResult::SucceededRouteEmpty,
            route::shortenRoute(createParaPoint(endLaneId, physics::ParametricValue(0.71)), route));
  compareRoutes(compareRoute, route);
  ASSERT_TRUE(route.roadSegments.empty());

  // nothing left
  ASSERT_EQ(route::ShortenRouteResult::FailedRouteEmpty,
            route::shortenRoute(createParaPoint(endLaneId, physics::ParametricValue(0.7)), route));

  // something totally outside
  route = routeResult;
  ASSERT_FALSE(route.roadSegments.empty());
  ASSERT_EQ(route::ShortenRouteResult::FailedRouteEmpty,
            route::shortenRoute(createParaPoint(lane::LaneId(123456), physics::ParametricValue(0.7)), route));
  ASSERT_TRUE(route.roadSegments.empty());

  // DontCutIntersectionAndPrependIfSucceededBeforeRoute mode
  // jump at end of second lane: ensure correct handling on degenerated handling at the beginning
  route = routeResult;
  compareRoute = routeResult;
  ASSERT_EQ(route::ShortenRouteResult::SucceededIntersectionNotCut,
            route::shortenRoute(createParaPoint(secondLaneId, physics::ParametricValue(0.)),
                                route,
                                ShortenRouteMode::DontCutIntersectionAndPrependIfSucceededBeforeRoute));
  eraseFirstSegment(compareRoute);
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(0.);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // enter intersection
  ASSERT_EQ(route::ShortenRouteResult::SucceededIntersectionNotCut,
            route::shortenRoute(createParaPoint(thirdLaneId, physics::ParametricValue(0.)),
                                route,
                                ShortenRouteMode::DontCutIntersectionAndPrependIfSucceededBeforeRoute));
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // further within intersection
  ASSERT_EQ(route::ShortenRouteResult::SucceededIntersectionNotCut,
            route::shortenRoute(createParaPoint(thirdLaneId, physics::ParametricValue(0.5)),
                                route,
                                ShortenRouteMode::DontCutIntersectionAndPrependIfSucceededBeforeRoute));
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // end of intersection
  ASSERT_EQ(route::ShortenRouteResult::SucceededIntersectionNotCut,
            route::shortenRoute(createParaPoint(thirdLaneId, physics::ParametricValue(1.)),
                                route,
                                ShortenRouteMode::DontCutIntersectionAndPrependIfSucceededBeforeRoute));
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // now actually leaving the intersection, but jump into the second one
  ASSERT_EQ(route::ShortenRouteResult::SucceededIntersectionNotCut,
            route::shortenRoute(createParaPoint(fourthLaneId, physics::ParametricValue(0.5)),
                                route,
                                ShortenRouteMode::DontCutIntersectionAndPrependIfSucceededBeforeRoute));
  eraseFirstSegment(compareRoute);
  eraseFirstSegment(compareRoute);
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start
    = compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.end;
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // finally leaving also the second intersection
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(createParaPoint(endLaneId, physics::ParametricValue(0.5)),
                                route,
                                ShortenRouteMode::DontCutIntersectionAndPrependIfSucceededBeforeRoute));
  eraseFirstSegment(compareRoute);
  eraseFirstSegment(compareRoute);
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(0.5);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);
}

TEST_F(RoutePlanningTest, route_following_multiple_choices)
{
  route::FullRoute routeResult = prepareTpkRoute();
  route::FullRoute route = routeResult;
  route::FullRoute compareRoute = routeResult;

  validateRouteConnections(route);

  // entering next segment, second choice is nearer
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(std::vector<ParaPoint>{createParaPoint(firstLaneId, physics::ParametricValue(.8)),
                                                       createParaPoint(firstLaneId, physics::ParametricValue(.5))},
                                route));
  ASSERT_GT(compareRoute.roadSegments.size(), 0u);
  eraseFirstSegment(compareRoute);
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.5);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);

  // entering next segment with the second choice, first choice keeps segment and is therefore nearer
  ASSERT_EQ(route::ShortenRouteResult::Succeeded,
            route::shortenRoute(std::vector<ParaPoint>{createParaPoint(firstLaneId, physics::ParametricValue(.98)),
                                                       createParaPoint(secondLaneId, physics::ParametricValue(.01))},
                                route));
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.98);
  compareRoutes(compareRoute, route);
  validateRouteConnections(route);
#if 0
  // entering future segments with multiple choices: nearest is 138, 0.5
  ASSERT_TRUE(route::shortenRoute(std::vector<ParaPoint>{createParaPoint(fifthLaneId, physics::ParametricValue(0.5)),
                                                         createParaPoint(fifthLaneId, physics::ParametricValue(0.4)),
                                                         createParaPoint(fifthLaneId, physics::ParametricValue(0.6)),
                                                         createParaPoint(fourthLaneId, physics::ParametricValue(0.5)),
                                                         createParaPoint(fourthLaneId, physics::ParametricValue(1.))},
                                  route));
  eraseFirstSegment(compareRoute);
  eraseFirstSegment(compareRoute);
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.5);
  compareRoutes(compareRoute, route);

  // calculation on segment with negative direction: nearest is 408, 0.8
  ASSERT_TRUE(route::shortenRoute(std::vector<ParaPoint>{createParaPoint(fifthLaneId, physics::ParametricValue(.6)),
                                                         createParaPoint(fifthLaneId, physics::ParametricValue(.5)),
                                                         createParaPoint(fifthLaneId, physics::ParametricValue(.8)),
                                                         createParaPoint(fifthLaneId, physics::ParametricValue(.8)),
                                                         createParaPoint(endLaneId, physics::ParametricValue(.7))},
                                  route));
  eraseFirstSegment(compareRoute);
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.8);
  compareRoutes(compareRoute, route);

  // ensure not existing segments don't influence and negative direction is ensured: nearest is 360, 0.4
  ASSERT_TRUE(
    route::shortenRoute(std::vector<ParaPoint>{createParaPoint(lane::LaneId(123456), physics::ParametricValue(.5)),
                                               createParaPoint(fifthLaneId, physics::ParametricValue(.4)),
                                               createParaPoint(lane::LaneId(123456), physics::ParametricValue(.7))},
                        route));
  compareRoute.roadSegments[0].drivableLaneSegments[0].laneInterval.start = physics::ParametricValue(.4);
  compareRoutes(compareRoute, route);

  // but if there are only not existing segments, then for sure, the route is cleaned
  ASSERT_FALSE(
    route::shortenRoute(std::vector<ParaPoint>{createParaPoint(lane::LaneId(123456), physics::ParametricValue(.5)),
                                               createParaPoint(lane::LaneId(123457), physics::ParametricValue(.7)),
                                               createParaPoint(lane::LaneId(123458), physics::ParametricValue(.7))},
                        route));
  compareRoute.roadSegments.clear();
  compareRoutes(compareRoute, route);
#endif
}

TEST_F(RoutePlanningTest, find_waypoint_result)
{
  route::FullRoute routeResult = prepareTpkRoute();

  auto findWaypointResult
    = route::findWaypoint(createParaPoint(firstLaneId, physics::ParametricValue(.5)), routeResult);

  uint64_t expectedSegmentCount = 6u;
  ASSERT_TRUE(findWaypointResult.isValid());

  lane::LaneId foundLaneId = findWaypointResult.laneSegmentIterator->laneInterval.laneId;
  route::SegmentCounter foundSegmentCounter = findWaypointResult.roadSegmentIterator->segmentCountFromDestination;
  ASSERT_EQ(foundLaneId, firstLaneId);
  ASSERT_EQ(foundSegmentCounter, expectedSegmentCount);

  auto leftLane = findWaypointResult.getLeftLane();
  ASSERT_FALSE(leftLane.isValid());

  auto rightLane = findWaypointResult.getRightLane();
  ASSERT_FALSE(rightLane.isValid());

  // iterate the successors
  expectedSegmentCount = findWaypointResult.roadSegmentIterator->segmentCountFromDestination - 1;
  for (auto successorLanes = findWaypointResult.getSuccessorLanes(); !successorLanes.empty();
       successorLanes = successorLanes[0].getSuccessorLanes(), expectedSegmentCount--)
  {
    ASSERT_TRUE(successorLanes[0].isValid());
    foundSegmentCounter = successorLanes[0].roadSegmentIterator->segmentCountFromDestination;
    ASSERT_EQ(foundSegmentCounter, expectedSegmentCount);
  }
  ASSERT_EQ(0u, expectedSegmentCount);

  // iterate the predecessors
  expectedSegmentCount = findWaypointResult.roadSegmentIterator->segmentCountFromDestination + 1;
  for (auto predecessorLanes = findWaypointResult.getPredecessorLanes(); !predecessorLanes.empty();
       predecessorLanes = predecessorLanes[0].getPredecessorLanes(), expectedSegmentCount++)
  {
    ASSERT_TRUE(predecessorLanes[0].isValid());
    foundSegmentCounter = predecessorLanes[0].roadSegmentIterator->segmentCountFromDestination;
    ASSERT_EQ(foundSegmentCounter, expectedSegmentCount);
  }
  ASSERT_EQ(8u, expectedSegmentCount);
}

TEST_F(RoutePlanningTest, get_route_section)
{
  route::FullRoute routeResult = prepareTpkRoute();
  validateRouteConnections(routeResult);

  route::FullRoute routeSection = route::getRouteSection(createParaPoint(firstLaneId, physics::ParametricValue(.5)),
                                                         physics::Distance(1.),
                                                         physics::Distance(1.),
                                                         routeResult);

  validateRouteConnections(routeSection);
  ASSERT_NEAR(2., double(route::calcLength(routeSection)), 0.1);

  routeSection = route::getRouteSection(createParaPoint(firstLaneId, physics::ParametricValue(.5)),
                                        physics::Distance(10.),
                                        physics::Distance(10.),
                                        routeResult);

  validateRouteConnections(routeSection);
  ASSERT_NEAR(20., double(route::calcLength(routeSection)), 0.1);

  routeSection = route::getRouteSection(createParaPoint(firstLaneId, physics::ParametricValue(.5)),
                                        physics::Distance(50.),
                                        physics::Distance(80.),
                                        routeResult);

  validateRouteConnections(routeSection);
  ASSERT_NEAR(130., double(route::calcLength(routeSection)), 0.1);

  routeSection = route::getRouteSection(createParaPoint(firstLaneId, physics::ParametricValue(.5)),
                                        physics::Distance(1000.),
                                        physics::Distance(1000.),
                                        routeResult);

  validateRouteConnections(routeSection);
  ASSERT_NEAR(195.3, double(route::calcLength(routeSection)), 0.1);
  compareRoutes(routeSection, routeResult);
}

TEST_F(RoutePlanningTest, route_planning_multi_lane_changes_OstringKaeppelstr)
{
  ASSERT_TRUE(access::init(mTestFile));

  startLaneId = lane::uniqueLaneId(createGeoPoint(Longitude(8.4396445), Latitude(49.0186033), Altitude(0.)));
  auto rightLaneId = lane::uniqueLaneId(createGeoPoint(Longitude(8.4394605), Latitude(49.0183941), Altitude(0.)));
  auto leftLaneId = lane::uniqueLaneId(createGeoPoint(Longitude(8.439489), Latitude(49.0183852), Altitude(0.)));

  route::FullRoute routeLeft = route::planning::planRoute(createParaPoint(startLaneId, physics::ParametricValue(0.2)),
                                                          createParaPoint(leftLaneId, physics::ParametricValue(.5)));
  route::FullRoute routeRight = route::planning::planRoute(createParaPoint(startLaneId, physics::ParametricValue(0.2)),
                                                           createParaPoint(rightLaneId, physics::ParametricValue(.5)));

  validateRouteConnections(routeLeft, RouteIsBroadening);
  validateRouteConnections(routeRight, RouteIsBroadening);
  compareRoutes(routeLeft, routeRight, EndSegmentDiffers);
}

TEST_F(RoutePlanningTest, extend_route_by_distance)
{
  ASSERT_TRUE(access::init(mTestFile));

  startLaneId = lane::uniqueLaneId(createGeoPoint(Longitude(8.4396613), Latitude(49.0185636), Altitude(0.)));

  route::FullRoute const initialRoute
    = route::planning::planRoute(createParaPoint(startLaneId, physics::ParametricValue(0.)),
                                 createParaPoint(startLaneId, physics::ParametricValue(.1)));

  auto const initialRouteLength = route::calcLength(initialRoute);

  auto route = initialRoute;
  std::vector<route::FullRoute> additionalRoutes;
  ASSERT_TRUE(extendRouteToDistance(route, initialRouteLength + physics::Distance(30.), additionalRoutes));
  // still one road segment and no additional routes, route segment expanded till end
  ASSERT_EQ(route.roadSegments.size(), 1u);
  ASSERT_TRUE(additionalRoutes.empty());
  auto routeLength = route::calcLength(route);
  ASSERT_GT(routeLength, initialRouteLength + physics::Distance(30.));
  ASSERT_EQ(route.roadSegments.front().drivableLaneSegments.front().laneInterval.laneId, startLaneId);
  ASSERT_EQ(route.roadSegments.front().drivableLaneSegments.front().laneInterval.start, physics::ParametricValue(0.));
  ASSERT_EQ(route.roadSegments.front().drivableLaneSegments.front().laneInterval.end, physics::ParametricValue(1.));
  validateRouteParaPoints(route);
  validateRouteConnections(route, RouteIsConstant);

  // extending further spans into T-intersection and leads to an additional route
  ASSERT_TRUE(extendRouteToDistance(route, routeLength + physics::Distance(30.), additionalRoutes));
  ASSERT_EQ(route.roadSegments.size(), 3u);
  ASSERT_EQ(additionalRoutes.size(), 1u);
  ASSERT_EQ(additionalRoutes.front().roadSegments.size(), 3u);
  auto routeLengthA = route::calcLength(route);
  auto routeLengthB = route::calcLength(additionalRoutes.front());
  ASSERT_GT(routeLengthA, routeLength + physics::Distance(30.));
  ASSERT_GT(routeLengthB, routeLength + physics::Distance(30.));
  // the begin of the routes is identical
  ASSERT_EQ(route.roadSegments.front().drivableLaneSegments.front().laneInterval.laneId,
            additionalRoutes.front().roadSegments.front().drivableLaneSegments.front().laneInterval.laneId);
  // the end not
  ASSERT_NE(route.roadSegments.back().drivableLaneSegments.front().laneInterval.laneId,
            additionalRoutes.back().roadSegments.front().drivableLaneSegments.front().laneInterval.laneId);
  validateRouteParaPoints(route);
  validateRouteConnections(route, RouteIsConstant);
  validateRouteParaPoints(additionalRoutes.back());
  validateRouteConnections(additionalRoutes.back(), RouteIsConstant);
}

TEST_F(RoutePlanningTest, extend_route_by_destination)
{
  ASSERT_TRUE(access::init(mTestFile));

  startLaneId = lane::uniqueLaneId(createGeoPoint(Longitude(8.4396613), Latitude(49.0185636), Altitude(0.)));

  route::FullRoute const initialRoute
    = route::planning::planRoute(createParaPoint(startLaneId, physics::ParametricValue(0.)),
                                 createParaPoint(startLaneId, physics::ParametricValue(.1)),
                                 RouteCreationMode::AllRoutableLanes);

  auto const initialRouteLength = route::calcLength(initialRoute);

  // extending by geo points
  auto furtherDownTheSameLane = createGeoPoint(Longitude(8.4400608), Latitude(49.0190709), Altitude(0.));
  auto route = initialRoute;
  ASSERT_TRUE(extendRouteToDestinations(route, {furtherDownTheSameLane}));
  // still one road segment and no additional routes, route segment expanded till end
  ASSERT_EQ(route.roadSegments.size(), 1u);
  auto routeLength = route::calcLength(route);
  ASSERT_GT(routeLength, initialRouteLength);
  ASSERT_EQ(route.roadSegments.front().drivableLaneSegments.front().laneInterval.laneId, startLaneId);
  ASSERT_EQ(route.roadSegments.front().drivableLaneSegments.front().laneInterval.start, physics::ParametricValue(0.));
  ASSERT_EQ(route.roadSegments.front().drivableLaneSegments.front().laneInterval.end,
            physics::ParametricValue(.847167));
  validateRouteParaPoints(route);
  validateRouteConnections(route, RouteIsConstant);

  // extending by furhter via some intersections geo points
  auto furtherAccrossNextIntersection = createGeoPoint(Longitude(8.4402588), Latitude(49.0193127), Altitude(0.));
  auto nearbyStartOppositeLane = createGeoPoint(Longitude(8.439657), Latitude(49.018624), Altitude(0.));
  route = initialRoute;
  ASSERT_TRUE(extendRouteToDestinations(
    route, {furtherDownTheSameLane, furtherAccrossNextIntersection, nearbyStartOppositeLane}));
  // still one road segment and no additional routes, route segment expanded till end
  ASSERT_GT(route.roadSegments.size(), 1u);
  routeLength = route::calcLength(route);
  ASSERT_GT(routeLength, initialRouteLength);

  ASSERT_EQ(route.roadSegments.front().drivableLaneSegments.front().laneInterval.laneId,
            route.roadSegments.back().drivableLaneSegments.back().laneInterval.laneId);
}

TEST_F(RoutePlanningTest, routing_point)
{
  ASSERT_TRUE(access::init(mTestFile));
  access::setENUReferencePoint(mTestPoints.back().first);

  auto lanes = lane::getLanes();
  ASSERT_GT(lanes.size(), 0u);
  lane::LaneId x11 = lanes[0];

  match::LaneOccupiedRegion occupiedRegion;
  occupiedRegion.laneId = x11;
  occupiedRegion.longitudinalRange.minimum = ::ad::physics::ParametricValue(0.2);
  occupiedRegion.longitudinalRange.maximum = ::ad::physics::ParametricValue(0.8);
  occupiedRegion.lateralRange.minimum = ::ad::physics::ParametricValue(0.5);
  occupiedRegion.lateralRange.maximum = ::ad::physics::ParametricValue(0.8);

  RoutingParaPoint routingPoint1, routingPoint2;
  routingPoint1 = createRoutingPoint(occupiedRegion, RoutingDirection::POSITIVE);
  ASSERT_EQ(routingPoint1.point.laneId, x11);
  ASSERT_DOUBLE_EQ((double)routingPoint1.point.parametricOffset, 0.2);
  ASSERT_EQ(routingPoint1.direction, RoutingDirection::POSITIVE);

  routingPoint2 = createRoutingPoint(occupiedRegion, point::createENUHeading(M_PI_2));
  ASSERT_EQ(routingPoint2.point.laneId, x11);
  ASSERT_DOUBLE_EQ((double)routingPoint2.point.parametricOffset, 0.8);
  ASSERT_EQ(routingPoint2.direction, RoutingDirection::NEGATIVE);
}

TEST_F(RoutePlanningTest, route_para_point)
{
  ASSERT_TRUE(access::init(mTestFile));
  access::setENUReferencePoint(mTestPoints.back().first);

  startLaneId = lane::uniqueLaneId(createGeoPoint(Longitude(8.4396445), Latitude(49.0186033), Altitude(0.)));
  auto leftLaneId = lane::uniqueLaneId(createGeoPoint(Longitude(8.439489), Latitude(49.0183852), Altitude(0.)));

  ParaPoint paraPoint1, paraPoint2;
  paraPoint1 = createParaPoint(startLaneId, physics::ParametricValue(0.2));
  paraPoint2 = createParaPoint(leftLaneId, physics::ParametricValue(.5));

  physics::Distance dis;
  route::FullRoute routeLeft, routeEmpty;

  routeLeft = route::planning::planRoute(paraPoint1, paraPoint2);
  validateRouteConnections(routeLeft, RouteIsBroadening);
  RouteParaPoint routeParaPoint1, routeParaPoint2;
  ASSERT_TRUE(getRouteParaPointFromParaPoint(paraPoint1, routeLeft, routeParaPoint1));
  ASSERT_TRUE(getRouteParaPointFromParaPoint(paraPoint2, routeLeft, routeParaPoint2));

  dis = ad::map::route::calcLength(routeParaPoint1, routeParaPoint2, routeLeft);
  ASSERT_NEAR((double)dis, 22.1776, 0.0001);

  FindWaypointResult findWaypointResult = route::findWaypoint(paraPoint2, routeLeft);
  ASSERT_TRUE(findWaypointResult.isValid());
  dis = ad::map::route::calcLength(findWaypointResult);
  ASSERT_NEAR((double)dis, 27.7613, 0.0001);

  findWaypointResult = findWaypoint(startLaneId, routeLeft);
  ASSERT_TRUE(findWaypointResult.isValid());

  physics::Duration duration;
  duration = calcDuration(routeLeft);
  ASSERT_NEAR((double)duration, 3.3313, 0.0001);
  ASSERT_TRUE(isWithinInterval(routeLeft.roadSegments[0], paraPoint1));

  RouteIterator routeIter0 = getRouteIterator(routeParaPoint1, routeEmpty);
  ASSERT_FALSE(routeIter0.isValid());
  RouteIterator routeIter1 = getRouteIterator(routeParaPoint1, routeLeft);
  RouteIterator routeIter2 = getRouteIterator(routeParaPoint2, routeLeft);
  ASSERT_TRUE(routeIter1.isValid());
  ASSERT_TRUE(routeIter2.isValid());
  dis = ad::map::route::calcLength(routeIter1, routeIter2);
  ASSERT_NEAR((double)dis, 27.7613, 0.0001);

  restriction::SpeedLimitList speedLimits;
  speedLimits = getSpeedLimits(routeIter1, routeIter2);
  ASSERT_EQ(speedLimits.size(), 1u);
  ASSERT_NEAR((double)speedLimits[0].speedLimit, 8.3333, 0.0001);
  ASSERT_NEAR((double)speedLimits[0].lanePiece.minimum, 0., 0.0001);
  ASSERT_NEAR((double)speedLimits[0].lanePiece.maximum, 1., 0.0001);

  speedLimits = getSpeedLimits(routeLeft);
  ASSERT_EQ(speedLimits.size(), 3u);
  for (auto const &speedLimit : speedLimits)
  {
    ASSERT_NEAR((double)speedLimit.speedLimit, 8.3333, 0.0001);
    ASSERT_NEAR((double)speedLimit.lanePiece.minimum, 0., 0.0001);
    ASSERT_NEAR((double)speedLimit.lanePiece.maximum, 1., 0.0001);
  }

  route::FullRoute bypassingRoute;
  ASSERT_TRUE(calculateBypassingRoute(routeLeft, bypassingRoute));
  duration = calcDuration(bypassingRoute);
  ASSERT_NEAR((double)duration, 3.3136, 0.0001);
  validateRouteConnections(bypassingRoute);

  RouteParaPoint resultingPoint;
  ASSERT_FALSE(
    calculateRouteParaPointAtDistance(routeEmpty, routeParaPoint1, physics::Distance(22.1776), resultingPoint));
  ASSERT_TRUE(
    calculateRouteParaPointAtDistance(routeLeft, routeParaPoint1, physics::Distance(22.1776), resultingPoint));
  ASSERT_EQ(routeParaPoint2.routePlanningCounter, resultingPoint.routePlanningCounter);
  ASSERT_EQ(routeParaPoint2.segmentCountFromDestination, resultingPoint.segmentCountFromDestination);
  ASSERT_NEAR((double)routeParaPoint2.parametricOffset, (double)resultingPoint.parametricOffset, 0.0001);

  route::FullRoute expandedRoute;
  expandedRoute = getRouteExpandedToAllNeighborLanes(routeLeft);
  dis = ad::map::route::calcLength(routeParaPoint1, routeParaPoint2, routeLeft);
  ASSERT_NEAR((double)dis, 22.1776, 0.0001);
  validateRouteConnections(expandedRoute, RouteIsBroadening);

  shortenRouteToDistance(routeLeft, physics::Distance(19.1776));
  dis = ad::map::route::calcLength(routeParaPoint1, routeParaPoint2, routeLeft);
  ASSERT_NEAR((double)dis, 18.0002, 0.0001);
  validateRouteConnections(routeLeft, RouteIsBroadening);

  dis = addOpposingLaneSegmentToRoadSegment(paraPoint1, physics::Distance(1.), routeLeft.roadSegments[0], routeLeft);
  ASSERT_NEAR((double)dis, -1., 0.0001);
  validateRouteConnections(routeLeft, RouteIsBroadening);

  RoadSegment roadSegment;
  EXPECT_THROW(shortenSegmentFromBegin(roadSegment, dis), std::runtime_error);
  EXPECT_THROW(shortenSegmentFromEnd(roadSegment, dis), std::runtime_error);
}

TEST_F(RoutePlanningTest, route_match_position)
{
  ASSERT_TRUE(access::init(mTestFile));
  access::setENUReferencePoint(mTestPoints.back().first);

  physics::Distance dis;
  route::FullRoute routeLeft;
  physics::Duration duration;

  startLaneId = lane::uniqueLaneId(createGeoPoint(Longitude(8.4396445), Latitude(49.0186033), Altitude(0.)));
  auto leftLaneId = lane::uniqueLaneId(createGeoPoint(Longitude(8.439489), Latitude(49.0183852), Altitude(0.)));

  ParaPoint paraPoint1, paraPoint2;
  paraPoint1 = createParaPoint(startLaneId, physics::ParametricValue(0.2));
  paraPoint2 = createParaPoint(leftLaneId, physics::ParametricValue(.5));

  routeLeft = route::planning::planRoute(paraPoint1, paraPoint2);
  validateRouteConnections(routeLeft, RouteIsBroadening);

  point::GeoPoint point_geo1, point_geo2;
  point_geo1 = point::createGeoPoint(Longitude(8.4396445), Latitude(49.0186033), Altitude(0.));
  point_geo2 = point::createGeoPoint(Longitude(8.439489), Latitude(49.0183852), Altitude(0.));
  match::AdMapMatching mapMatching;
  match::MapMatchedPositionConfidenceList matchPosition1, matchPosition2;
  matchPosition1 = mapMatching.getMapMatchedPositions(point_geo1, physics::Distance(1), physics::Probability(0.5));
  matchPosition2 = mapMatching.getMapMatchedPositions(point_geo2, physics::Distance(1), physics::Probability(0.5));
  ASSERT_GT(matchPosition1.size(), 0u);
  ASSERT_GT(matchPosition2.size(), 0u);

  dis = signedDistanceToLane(startLaneId, routeLeft, matchPosition1);
  ASSERT_NEAR((double)dis, 0., 0.0001);
  dis = signedDistanceToLane(leftLaneId, routeLeft, matchPosition2);
  ASSERT_NEAR((double)dis, 0., 0.0001);

  FindWaypointResult findWaypointResult = findNearestWaypoint(matchPosition1, routeLeft);
  ASSERT_TRUE(findWaypointResult.isValid());

  point::ENUHeading heading1, heading2;
  heading1 = mapMatching.getLaneENUHeading(matchPosition1.front());
  heading2 = mapMatching.getLaneENUHeading(matchPosition2.front());

  match::ENUObjectPosition mObjectPosition1, mObjectPosition2;
  mObjectPosition1.centerPoint = point::toENU(point_geo1);
  mObjectPosition1.heading = heading1;
  mObjectPosition1.dimension.width = physics::Distance(1.);
  mObjectPosition1.dimension.length = physics::Distance(1.);
  mObjectPosition1.dimension.height = physics::Distance(0.5);
  mObjectPosition1.enuReferencePoint = access::getENUReferencePoint();
  mObjectPosition2.centerPoint = point::toENU(point_geo2);
  mObjectPosition2.heading = heading2;
  mObjectPosition2.dimension.width = physics::Distance(1.);
  mObjectPosition2.dimension.length = physics::Distance(1.);
  mObjectPosition2.dimension.height = physics::Distance(0.5);
  mObjectPosition2.enuReferencePoint = access::getENUReferencePoint();

  match::MapMatchedObjectBoundingBox boundBox1, boundBox2;
  boundBox1 = mapMatching.getMapMatchedBoundingBox(mObjectPosition1, physics::Distance(1));
  boundBox2 = mapMatching.getMapMatchedBoundingBox(mObjectPosition2, physics::Distance(1));
  ASSERT_GT(boundBox1.laneOccupiedRegions.size(), 0u);
  ASSERT_GT(boundBox2.laneOccupiedRegions.size(), 0u);

  std::vector<FullRoute> fullRouteVec;
  fullRouteVec = predictRoutesOnDistance(boundBox1, physics::Distance(22.1776));
  ASSERT_GT(fullRouteVec.size(), 0u);
  for (auto const &route : fullRouteVec)
  {
    validateRouteConnections(route, RouteIsBroadening);
  }

  fullRouteVec = predictRoutesOnDuration(boundBox1, physics::Duration(3.33));
  ASSERT_GT(fullRouteVec.size(), 0u);
  for (auto const &route : fullRouteVec)
  {
    validateRouteConnections(route, RouteIsBroadening);
  }

  auto findWaypointResult2 = route::objectOnRoute(boundBox1, fullRouteVec[0]);
  ASSERT_TRUE(findWaypointResult2.isValid());

  route::FullRoute routeResult;
  routeResult = getRouteExpandedToOppositeLanes(routeLeft);
  validateRouteConnections(routeResult, RouteIsBroadening);
  ASSERT_FALSE(addOpposingLaneToRoute(paraPoint1, physics::Distance(1.), routeResult, dis));
  ASSERT_FALSE(addOpposingLaneToRoute(paraPoint2, physics::Distance(10.), routeResult, dis));

  ParaPoint paraOppoPoint1;
  auto startLaneId1 = lane::uniqueLaneId(createGeoPoint(Longitude(8.4404330), Latitude(49.0195249), Altitude(0.)));
  auto rightLaneId1 = lane::uniqueLaneId(createGeoPoint(Longitude(8.4408261), Latitude(49.0200051), Altitude(0.)));
  auto oppositeLaneId = lane::uniqueLaneId(createGeoPoint(Longitude(8.4403594), Latitude(49.0194988), Altitude(0.)));

  paraPoint1 = createParaPoint(startLaneId1, physics::ParametricValue(0.2));
  paraPoint2 = createParaPoint(rightLaneId1, physics::ParametricValue(.5));
  routeLeft = route::planning::planRoute(paraPoint1, paraPoint2);
  validateRouteConnections(routeLeft);
  paraOppoPoint1 = createParaPoint(oppositeLaneId, physics::ParametricValue(0.9));
  ASSERT_TRUE(addOpposingLaneToRoute(paraOppoPoint1, physics::Distance(100.), routeLeft, dis));
  validateRouteConnections(routeLeft, RouteIsNarrowing);
  auto findWaypointResultOppo1 = route::findWaypoint(oppositeLaneId, routeLeft);
  ASSERT_TRUE(findWaypointResultOppo1.isValid());
  dis = calcLength(findWaypointResultOppo1);
  ASSERT_NEAR((double)dis, 0., 0.0001);
  auto findWaypointResultRight = route::findWaypoint(rightLaneId1, routeLeft);
  ASSERT_TRUE(findWaypointResultRight.isValid());
  dis = calcLength(findWaypointResultRight);
  ASSERT_NEAR((double)dis, 15.4080, 0.0001);
}

TEST_F(RoutePlanningTest, route_plan_given_geo)
{
  ASSERT_TRUE(access::init(mTestFile));
  access::setENUReferencePoint(mTestPoints.back().first);

  point::GeoPoint point_geo1, point_geo2, point_geo3;
  point_geo1 = point::createGeoPoint(Longitude(8.4396445), Latitude(49.0186033), Altitude(0.));
  point_geo2 = point::createGeoPoint(Longitude(8.439489), Latitude(49.0183852), Altitude(0.));

  startLaneId = lane::uniqueLaneId(point_geo1);
  auto leftLaneId = lane::uniqueLaneId(point_geo2);

  RoutingParaPoint startRouting;
  startRouting = createRoutingPoint(startLaneId, physics::ParametricValue(0.1), RoutingDirection::DONT_CARE);
  std::vector<point::GeoPoint> dest;
  dest.push_back(point_geo2);
  route::FullRoute routeLeft, routeResult;
  routeLeft = planRoute(startRouting, dest);

  ParaPoint paraPoint1, paraPoint2;
  paraPoint1 = createParaPoint(startLaneId, physics::ParametricValue(0.2));
  paraPoint2 = createParaPoint(leftLaneId, physics::ParametricValue(.5));

  physics::Distance dis;
  FindWaypointResult findWaypointResult1 = route::findWaypoint(paraPoint2, routeLeft);
  ASSERT_TRUE(findWaypointResult1.isValid());
  dis = ad::map::route::calcLength(findWaypointResult1);
  ASSERT_NEAR((double)dis, 19.4644, 0.0001);

  routeLeft = planRoute(startRouting, point_geo2);
  FindWaypointResult findWaypointResult2 = route::findWaypoint(paraPoint2, routeLeft);
  ASSERT_TRUE(findWaypointResult2.isValid());
  dis = ad::map::route::calcLength(findWaypointResult2);
  ASSERT_NEAR((double)dis, 19.4644, 0.0001);

  point_geo1 = point::createGeoPoint(Longitude(8.4396445), Latitude(49.0186033), Altitude(0.));
  auto startLaneId1 = lane::uniqueLaneId(point_geo1);
  startRouting = createRoutingPoint(startLaneId1, physics::ParametricValue(0.1), RoutingDirection::DONT_CARE);
  point_geo2 = point::createGeoPoint(Longitude(8.440442), Latitude(49.0199380), Altitude(0.));
  dest.clear();
  dest.push_back(point_geo2);
  routeLeft = planRoute(startRouting, dest);
  ASSERT_EQ(routeLeft, route::FullRoute());

  point_geo2 = point::createGeoPoint(Longitude(8.4404311), Latitude(49.0199250), Altitude(0.));
  dest.push_back(point_geo2);
  routeLeft = planRoute(startRouting, dest);
  ASSERT_EQ(routeLeft, route::FullRoute());

  dest.clear();
  point_geo1 = point::createGeoPoint(Longitude(8.4404844), Latitude(49.0195607), Altitude(0.));
  point_geo2 = point::createGeoPoint(Longitude(8.4406460), Latitude(49.0197543), Altitude(0.));
  point_geo3 = point::createGeoPoint(Longitude(8.4407874), Latitude(49.0199274), Altitude(0.));
  dest.push_back(point_geo2);
  dest.push_back(point_geo3);
  auto startLaneId3 = lane::uniqueLaneId(point_geo1);
  auto leftLaneId3 = lane::uniqueLaneId(point_geo3);
  startRouting = createRoutingPoint(startLaneId3, physics::ParametricValue(0.1), RoutingDirection::DONT_CARE);
  routeLeft = planRoute(startRouting, dest);
  paraPoint2 = createParaPoint(leftLaneId3, physics::ParametricValue(.5));
  FindWaypointResult findWaypointResult3 = route::findWaypoint(leftLaneId3, routeLeft);
  ASSERT_TRUE(findWaypointResult3.isValid());
  dis = ad::map::route::calcLength(findWaypointResult3);
  ASSERT_NEAR((double)dis, 71.7440, 0.0001);
}

TEST_F(RoutePlanningTest, connecting_route)
{
  ASSERT_TRUE(access::init("test_files/TPK_PFZ.adm.txt"));

  match::AdMapMatching mapMatching;

  physics::Dimension3D vehicleDimension;
  vehicleDimension.length = physics::Distance(5.0);
  vehicleDimension.width = physics::Distance(2.0);
  vehicleDimension.height = physics::Distance(1.5);

  // Emmy-Noether last section before intersection to Ada-Lovelace
  auto const emmyToAda = lane::uniqueLaneId(createGeoPoint(Longitude(8.4381157), Latitude(49.0208767), Altitude(0.)));
  auto const opositeToEmmyToAda = getContactLanes(lane::getLane(emmyToAda), lane::ContactLocation::LEFT).front().toLane;

  // Hirtenweg to Haid-und-Neu Str.
  auto const hirtToHaid = lane::uniqueLaneId(createGeoPoint(Longitude(8.4401012), Latitude(49.0181061), Altitude(0.)));
  // Haid-und-Neu Str. crossing Hirtenweg to Haid-und-Neu Str.
  auto const haidXhirtXHaid
    = lane::uniqueLaneId(createGeoPoint(Longitude(8.4404876), Latitude(49.0179614), Altitude(0.)));

  enum
  {
    EGO_LANEID,
    EGO_LANE_OFFSET_LON,
    EGO_LANE_OFFSET_LAT,
    EGO_HEADING_OFFSET,
    OTHER_LANEID,
    OTHER_LANE_OFFSET_LON,
    OTHER_LANE_OFFSET_LAT,
    OTHER_HEADING_OFFSET,
    MAX_DIST_TOO_SHORT,
    MAX_DIST_NORMAL,
    RESULT_TYPE,
    RESULT_ROUTE_EGO_LENGTH,
    RESULT_ROUTE_OTHER_LENGTH
  };

  std::vector<std::tuple<
    // ego positioning within the lane (paraPoint, lateralOffset, delta lane heading)
    lane::LaneId,
    physics::ParametricValue,
    physics::ParametricValue,
    double,
    // object positioning within the lane (paraPoint, lateralOffset, delta lane heading)
    lane::LaneId,
    physics::ParametricValue,
    physics::ParametricValue,
    double,
    // calculate connected route distances: too short (invalid result), normal
    physics::Distance,
    physics::Distance,
    // expected connecting route results: type, routeA length, routeB length
    route::ConnectingRouteType,
    physics::Distance,
    physics::Distance>>
    testEntries{
      // simple standard: directly in front same direction, same lane
      std::make_tuple(emmyToAda,
                      physics::ParametricValue(0.5),
                      physics::ParametricValue(0.5),
                      0.,
                      emmyToAda,
                      physics::ParametricValue(0.8),
                      physics::ParametricValue(0.5),
                      0.,
                      physics::Distance(0.),
                      physics::Distance(50.),
                      route::ConnectingRouteType::Following,
                      physics::Distance(4.33336),
                      physics::Distance(0.)),
      // simple standard: directly at back same direction, same lane
      std::make_tuple(emmyToAda,
                      physics::ParametricValue(0.5),
                      physics::ParametricValue(0.5),
                      0.,
                      emmyToAda,
                      physics::ParametricValue(0.2),
                      physics::ParametricValue(0.5),
                      0.,
                      physics::Distance(0.),
                      physics::Distance(50.),
                      route::ConnectingRouteType::Following,
                      physics::Distance(0.),
                      physics::Distance(4.32631)),
      // simple standard: directly in front other direction, other lane
      std::make_tuple(emmyToAda,
                      physics::ParametricValue(0.5),
                      physics::ParametricValue(0.5),
                      0.,
                      opositeToEmmyToAda,
                      physics::ParametricValue(0.8),
                      physics::ParametricValue(0.5),
                      0.,
                      physics::Distance(0.),
                      physics::Distance(50.),
                      route::ConnectingRouteType::Opposing,
                      physics::Distance(4.34512),
                      physics::Distance(4.34421)),
      // simple standard: already at back other direction, other lane
      std::make_tuple(emmyToAda,
                      physics::ParametricValue(0.5),
                      physics::ParametricValue(0.5),
                      0.,
                      opositeToEmmyToAda,
                      physics::ParametricValue(0.2),
                      physics::ParametricValue(0.5),
                      0.,
                      physics::Distance(0.),
                      physics::Distance(50.),
                      route::ConnectingRouteType::Invalid,
                      physics::Distance(0.),
                      physics::Distance(0.)),
      // simple standard: Y connected route
      std::make_tuple(hirtToHaid,
                      physics::ParametricValue(0.2),
                      physics::ParametricValue(0.5),
                      0.,
                      haidXhirtXHaid,
                      physics::ParametricValue(0.2),
                      physics::ParametricValue(0.5),
                      0.,
                      physics::Distance(5.),
                      physics::Distance(50.),
                      route::ConnectingRouteType::Merging,
                      physics::Distance(53.5544),
                      physics::Distance(35.5292)),
    };

  for (auto const &testEntry : testEntries)
  {
    match::Object egoObject;
    point::ParaPoint egoParaPoint
      = point::createParaPoint(std::get<EGO_LANEID>(testEntry), std::get<EGO_LANE_OFFSET_LON>(testEntry));
    egoObject.enuPosition.centerPoint = lane::getENULanePoint(egoParaPoint, std::get<EGO_LANE_OFFSET_LAT>(testEntry));
    egoObject.enuPosition.heading = point::createENUHeading(static_cast<double>(lane::getLaneENUHeading(egoParaPoint))
                                                            + std::get<EGO_HEADING_OFFSET>(testEntry));
    egoObject.enuPosition.dimension = vehicleDimension;
    egoObject.enuPosition.enuReferencePoint = access::getENUReferencePoint();
    egoObject.mapMatchedBoundingBox
      = mapMatching.getMapMatchedBoundingBox(egoObject.enuPosition, physics::Distance(1.));

    match::Object otherObject;
    point::ParaPoint otherParaPoint
      = point::createParaPoint(std::get<OTHER_LANEID>(testEntry), std::get<OTHER_LANE_OFFSET_LON>(testEntry));
    otherObject.enuPosition.centerPoint
      = lane::getENULanePoint(otherParaPoint, std::get<OTHER_LANE_OFFSET_LAT>(testEntry));
    otherObject.enuPosition.heading = point::createENUHeading(
      static_cast<double>(lane::getLaneENUHeading(otherParaPoint)) + std::get<OTHER_HEADING_OFFSET>(testEntry));
    otherObject.enuPosition.dimension = vehicleDimension;
    otherObject.enuPosition.enuReferencePoint = access::getENUReferencePoint();
    otherObject.mapMatchedBoundingBox
      = mapMatching.getMapMatchedBoundingBox(otherObject.enuPosition, physics::Distance(2.));

    auto const tooShortDistance = std::get<MAX_DIST_TOO_SHORT>(testEntry);
    ConnectingRoute connRoute = route::planning::calculateConnectingRoute(egoObject, otherObject, tooShortDistance);
    EXPECT_EQ(route::ConnectingRouteType::Invalid, connRoute.type);
    EXPECT_EQ(physics::Distance::getMax(), calcLength(connRoute));

    auto const standardDistance = std::get<MAX_DIST_NORMAL>(testEntry);
    connRoute = route::planning::calculateConnectingRoute(egoObject, otherObject, standardDistance);
    EXPECT_EQ(std::get<RESULT_TYPE>(testEntry), connRoute.type);
    RouteBehaviour routeBehavior = RouteBehaviour::RouteIsConstant;
    if (route::ConnectingRouteType::Merging == connRoute.type)
    {
      EXPECT_EQ(connRoute.routeA.roadSegments.back().drivableLaneSegments.size(),
                connRoute.routeB.roadSegments.back().drivableLaneSegments.size());
      for (auto i = 0u; i < connRoute.routeA.roadSegments.back().drivableLaneSegments.size(); i++)
      {
        EXPECT_TRUE(route::isDegenerated(connRoute.routeA.roadSegments.back().drivableLaneSegments[i].laneInterval));
        EXPECT_EQ(connRoute.routeA.roadSegments.back().drivableLaneSegments[i].laneInterval,
                  connRoute.routeB.roadSegments.back().drivableLaneSegments[i].laneInterval);
      }
      routeBehavior = RouteBehaviour::RouteIsBroadening;
    }
    EXPECT_EQ(std::get<RESULT_ROUTE_EGO_LENGTH>(testEntry), calcLength(connRoute.routeA));
    EXPECT_EQ(std::get<RESULT_ROUTE_OTHER_LENGTH>(testEntry), calcLength(connRoute.routeB));
    validateRouteConnections(connRoute.routeA, routeBehavior);
    validateRouteConnections(connRoute.routeB, routeBehavior);
    validateRouteParaPoints(connRoute.routeA);
    validateRouteParaPoints(connRoute.routeB);
  }
}
