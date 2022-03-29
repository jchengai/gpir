// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/access/Operation.hpp>
#include <ad/map/match/AdMapMatching.hpp>
#include <ad/map/route/Planning.hpp>
#include <ad/map/route/RouteOperation.hpp>
#include <algorithm>

#include <gtest/gtest.h>

using namespace ::ad;
using namespace ::ad::map;

struct LaneChangeTest : ::testing::Test
{
  LaneChangeTest()
  {
  }

  virtual ~LaneChangeTest() = default;

  virtual void SetUp()
  {
    loadTestFile();
  }

  virtual void TearDown()
  {
    ad::map::access::cleanup();
  }

  match::MapMatchedPosition getPOIMapMatchedPosition(std::string const poiName) const;

  /* mTestFile looks like the following:
   *
   *                          f
   *                         _f
   *                        e
   *                       e
   * aaaa|bbbb|cccc|dddd|ee
   *     |bbbb|    |dddd|gggg|hhhh
   *
   * transitions: (a->b), (b->c) (c->d) (d->e) and (d->f)
   * route points:
   * - A: a point on road segment a
   * - BL: a point on the left lane of road segment b
   * - BR: a point on the right lane of road segment b
   * - F: a point on road segment f
   * - H: a point on road segment h
   * the lane change always happens on road section d
   *
   *
   */
  std::string mTestFile{"test_files/LaneChange.adm.txt"};

  void loadTestFile();

  lane::ContactLaneList getSuccessorsAndPredecessorsForLaneId(lane::LaneId const &laneId) const;

  void testsLaneChangeEnd(route::FindLaneChangeResult const &findLaneChangeResult, int expectedEndIndex) const;

  void testsLaneChangeStart(route::FindLaneChangeResult const &findLaneChangeResult, int expectedStartIndex) const;
};

void LaneChangeTest::loadTestFile()
{
  ASSERT_TRUE(ad::map::access::init(mTestFile));

  auto mapCenterPoint = point::GeoPoint();
  mapCenterPoint.latitude = point::Latitude(49.0183631);
  mapCenterPoint.longitude = point::Longitude(8.4411676);
  mapCenterPoint.altitude = point::Altitude(0.0);

  ad::map::access::setENUReferencePoint(mapCenterPoint);
  ASSERT_EQ(5u, ad::map::access::getPointsOfInterest().size());
}

match::MapMatchedPosition LaneChangeTest::getPOIMapMatchedPosition(std::string const poiName) const
{
  match::MapMatchedPosition invalidPoint;

  auto foundPoi
    = std::find_if(std::begin(ad::map::access::getPointsOfInterest()),
                   std::end(ad::map::access::getPointsOfInterest()),
                   [&poiName](config::PointOfInterest const &poi) { return poi.name.compare(poiName) == 0; });
  EXPECT_NE(foundPoi, std::end(ad::map::access::getPointsOfInterest()));
  if (foundPoi == std::end(ad::map::access::getPointsOfInterest()))
  {
    return invalidPoint;
  }

  ad::map::match::AdMapMatching mapMatching;
  match::MapMatchedPositionConfidenceList mapMatchingResult
    = mapMatching.getMapMatchedPositions(foundPoi->geoPoint, ::ad::physics::Distance(0.2), physics::Probability(.8));
  EXPECT_EQ(1u, mapMatchingResult.size()) << " requested POI: " << poiName;
  if (1u != mapMatchingResult.size())
  {
    return invalidPoint;
  }

  return mapMatchingResult.front();
}

void LaneChangeTest::testsLaneChangeEnd(route::FindLaneChangeResult const &findLaneChangeResult,
                                        int expectedEndIndex) const
{
  ASSERT_TRUE(findLaneChangeResult.isValid());

  ASSERT_EQ(expectedEndIndex,
            std::distance(std::begin(findLaneChangeResult.queryRoute.roadSegments),
                          findLaneChangeResult.laneChangeEndRouteIterator));

  // The end of the lane change cannot be the end of the route, otherwise we would not have a "next" lane to change to
  ASSERT_NE(findLaneChangeResult.laneChangeEndRouteIterator,
            std::prev(findLaneChangeResult.queryRoute.roadSegments.end()));
}

void LaneChangeTest::testsLaneChangeStart(route::FindLaneChangeResult const &findLaneChangeResult,
                                          int expectedStartIndex) const
{
  // The lane change start is never at the end
  ASSERT_NE(findLaneChangeResult.laneChangeStartRouteIterator, std::end(findLaneChangeResult.queryRoute.roadSegments));

  ASSERT_EQ(expectedStartIndex,
            std::distance(std::begin(findLaneChangeResult.queryRoute.roadSegments),
                          findLaneChangeResult.laneChangeStartRouteIterator));
}

TEST_F(LaneChangeTest, current_position_not_on_route)
{
  auto mapMatchedPositionStart = getPOIMapMatchedPosition("A");
  // ASSERT_NE(match::MapMatchedPosition(), mapMatchedPositionStart);

  auto mapMatchedPositionEnd = getPOIMapMatchedPosition("H");
  // ASSERT_NE(match::MapMatchedPosition(), mapMatchedPositionEnd);

  auto currentPosition = getPOIMapMatchedPosition("F");
  // ASSERT_NE(match::MapMatchedPosition(), currentPosition);

  auto route = route::planning::planRoute(mapMatchedPositionStart.lanePoint.paraPoint,
                                          mapMatchedPositionEnd.lanePoint.paraPoint);

  // This function needs to be tested:
  auto findLaneChangeResult = route::findFirstLaneChange(currentPosition, route);

  // F is not on the route - the result is invalid
  ASSERT_FALSE(findLaneChangeResult.isValid());
  ASSERT_EQ(&findLaneChangeResult.queryRoute, &route);
  ASSERT_EQ(findLaneChangeResult.laneChangeStartRouteIterator, std::end(route.roadSegments));
  ASSERT_EQ(findLaneChangeResult.laneChangeEndRouteIterator, std::end(route.roadSegments));
  ASSERT_EQ(findLaneChangeResult.laneChangeDirection, route::LaneChangeDirection::Invalid);
}

TEST_F(LaneChangeTest, find_lane_change_A_to_H_start_at_A)
{
  auto mapMatchedPositionStart = getPOIMapMatchedPosition("A");
  // ASSERT_NE(match::MapMatchedPosition(), mapMatchedPositionStart);

  auto mapMatchedPositionEnd = getPOIMapMatchedPosition("H");
  // ASSERT_NE(match::MapMatchedPosition(), mapMatchedPositionEnd);

  auto route = route::planning::planRoute(mapMatchedPositionStart.lanePoint.paraPoint,
                                          mapMatchedPositionEnd.lanePoint.paraPoint);

  // We assume our car is on the start point A
  auto currentPosition = mapMatchedPositionStart;

  // This function needs to be tested:
  auto findLaneChangeResult = route::findFirstLaneChange(currentPosition, route);

  // The query route should be the route given
  ASSERT_EQ(&findLaneChangeResult.queryRoute, &route);

  // The end of the lane change is at index 3 (road segment d)
  testsLaneChangeEnd(findLaneChangeResult, 3);

  // The start of the lane change is at index 3
  testsLaneChangeStart(findLaneChangeResult, 3);

  // Check if the lane change was to the left
  ASSERT_EQ(findLaneChangeResult.laneChangeDirection, route::LaneChangeDirection::LeftToRight);

  // There is a lane change on the route, so the returned result is valid
  ASSERT_TRUE(findLaneChangeResult.isValid());
}

TEST_F(LaneChangeTest, find_lane_change_A_to_F_start_at_A)
{
  auto mapMatchedPositionStart = getPOIMapMatchedPosition("A");
  // ASSERT_NE(match::MapMatchedPosition(), mapMatchedPositionStart);

  auto mapMatchedPositionEnd = getPOIMapMatchedPosition("F");
  // ASSERT_NE(match::MapMatchedPosition(), mapMatchedPositionEnd);

  auto route = route::planning::planRoute(mapMatchedPositionStart.lanePoint.paraPoint,
                                          mapMatchedPositionEnd.lanePoint.paraPoint);

  // Set our car on the start of the route
  auto currentPosition = mapMatchedPositionStart;

  // This function needs to be tested:
  auto findLaneChangeResult = route::findFirstLaneChange(currentPosition, route);

  // The query route should be the route given
  ASSERT_EQ(&findLaneChangeResult.queryRoute, &route);

  // There is no lane change on the route, so the boost optional is not set
  ASSERT_FALSE(findLaneChangeResult.isValid());

  // The start of the lane change set to invalid (std::end(route.roadSegments))
  ASSERT_EQ(findLaneChangeResult.laneChangeStartRouteIterator, std::end(route.roadSegments));

  // The end of the lane change is set to invalid (std::end(route.roadSegments))
  ASSERT_EQ(findLaneChangeResult.laneChangeEndRouteIterator, std::end(route.roadSegments));

  ASSERT_EQ(findLaneChangeResult.laneChangeDirection, route::LaneChangeDirection::Invalid);
}

TEST_F(LaneChangeTest, find_lane_change_A_to_F_start_at_BL)
{
  auto mapMatchedPositionStart = getPOIMapMatchedPosition("A");
  // ASSERT_NE(match::MapMatchedPosition(), mapMatchedPositionStart);

  auto mapMatchedPositionEnd = getPOIMapMatchedPosition("F");
  // ASSERT_NE(match::MapMatchedPosition(), mapMatchedPositionEnd);

  auto currentPosition = getPOIMapMatchedPosition("BL");

  auto route = route::planning::planRoute(mapMatchedPositionStart.lanePoint.paraPoint,
                                          mapMatchedPositionEnd.lanePoint.paraPoint);

  // This function needs to be tested:
  auto findLaneChangeResult = route::findFirstLaneChange(currentPosition, route);

  // The query route should be the route given
  ASSERT_EQ(&findLaneChangeResult.queryRoute, &route);

  // There is no lane change on the route
  ASSERT_FALSE(findLaneChangeResult.isValid());
  ASSERT_EQ(findLaneChangeResult.laneChangeStartRouteIterator, std::end(route.roadSegments));
  ASSERT_EQ(findLaneChangeResult.laneChangeEndRouteIterator, std::end(route.roadSegments));
  ASSERT_EQ(findLaneChangeResult.laneChangeDirection, route::LaneChangeDirection::Invalid);
}

TEST_F(LaneChangeTest, find_lane_change_BL_to_F_start_at_BR)
{
  auto mapMatchedPositionStart = getPOIMapMatchedPosition("BL");
  // ASSERT_NE(match::MapMatchedPosition(), mapMatchedPositionStart);
  auto mapMatchedPositionEnd = getPOIMapMatchedPosition("F");
  // ASSERT_NE(match::MapMatchedPosition(), mapMatchedPositionEnd);
  auto currentPosition = getPOIMapMatchedPosition("BR");
  // ASSERT_NE(match::MapMatchedPosition(), currentPosition);

  auto route = route::planning::planRoute(mapMatchedPositionStart.lanePoint.paraPoint,
                                          mapMatchedPositionEnd.lanePoint.paraPoint);

  // This function needs to be tested:
  auto findLaneChangeResult = route::findFirstLaneChange(currentPosition, route);

  // The query route should be the route given
  ASSERT_EQ(&findLaneChangeResult.queryRoute, &route);

  // The lane change ends at the very first segment (happens on b only)
  testsLaneChangeEnd(findLaneChangeResult, 0);

  testsLaneChangeStart(findLaneChangeResult, 0);

  // There is a lane change on the route from (b left to b right)
  ASSERT_TRUE(findLaneChangeResult.isValid());

  ASSERT_EQ(findLaneChangeResult.laneChangeDirection, route::LaneChangeDirection::RightToLeft);

  physics::Distance dis;
  dis = findLaneChangeResult.calcZoneLength();
  ASSERT_NEAR((double)dis, 8.2828, 0.0001);
}
