// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/access/Operation.hpp>
#include <ad/map/lane/LaneOperation.hpp>
#include <ad/map/match/AdMapMatching.hpp>
#include <ad/map/test_support/NoLogTestMacros.hpp>
#include <gtest/gtest.h>

using namespace ::ad;
using namespace ::ad::map;
using namespace ::ad::map::match;

struct AdMapMatchingTest : ::testing::Test
{
  AdMapMatchingTest()
  {
  }

  virtual void SetUp()
  {
    access::cleanup();
    access::init("test_files/TPK.adm.txt");
    mRouteHint = route::FullRoute();
    mMinProbabilty = physics::Probability(0.05);
    mHeadingHints.clear();
    mMapMatching = new match::AdMapMatching();
  }

  virtual void TearDown()
  {
    delete mMapMatching;
    access::cleanup();
  }

  void compareMapMatching(int line, point::GeoPoint const &geoPoint, std::size_t const expectedNumberOfMatches);

  void addRouteHint()
  {
    // with route hint straigth through Albert-Nestler-Str
    auto hintPoint
      = point::createGeoPoint(point::Longitude(8.4401803), point::Latitude(49.0191987), point::Altitude(0.));
    auto mapMatchingResults = mMapMatching->getMapMatchedPositions(hintPoint, physics::Distance(0.01), mMinProbabilty);
    ASSERT_EQ(mapMatchingResults.size(), 1u);
    route::RoadSegment roadSegment;
    route::LaneSegment laneSegment;
    laneSegment.laneInterval.laneId = mapMatchingResults.front().lanePoint.paraPoint.laneId;
    laneSegment.laneInterval.start = physics::ParametricValue(0.);
    laneSegment.laneInterval.end = physics::ParametricValue(1.);
    roadSegment.drivableLaneSegments.push_back(laneSegment);
    mRouteHint.roadSegments.push_back(roadSegment);
  }

  /* A note on the test setup:
   * Location is the intersection Albert-Nestler-Str crossing Ada-Lovelace-Str
   *
   * Given:
   *            |  -> Albert-Nestler-Str
   *            |
   * -----------+
   *  ^         |
   *  |         |  -> Albert-Nestler-Str
   *  |
   *  Ada-Lovel.
   *
   * All points for matching (mTestPoints) should be along the lane from south to north
   * inside the intersection, e.g.
   * 8.4401468/49.0191790
   * 8.4401701/49.0192037
   * 8.4401889/49.0192335
   * are all along that lane (ordered from south to north)
   */

  std::string mTestFile{"test_files/TPK.adm.txt"};
  std::vector<std::pair<point::GeoPoint, size_t>> mTestPoints{
    std::make_pair(point::createGeoPoint(point::Longitude(8.4400665), point::Latitude(49.0192005), point::Altitude(0.)),
                   0u),
    std::make_pair(point::createGeoPoint(point::Longitude(8.4401882), point::Latitude(49.0191939), point::Altitude(0.)),
                   1u),
    std::make_pair(point::createGeoPoint(point::Longitude(8.4401510), point::Latitude(49.0191792), point::Altitude(0.)),
                   2u),
    std::make_pair(point::createGeoPoint(point::Longitude(8.4401742), point::Latitude(49.0192009), point::Altitude(0.)),
                   3u),
    // approaching middle of the intersection
    std::make_pair(point::createGeoPoint(point::Longitude(8.4401540), point::Latitude(49.0192082), point::Altitude(0.)),
                   4u)};

  route::FullRoute mRouteHint;
  std::vector<point::ENUHeading> mHeadingHints;
  physics::Probability mMinProbabilty;
  match::AdMapMatching *mMapMatching;
};

void AdMapMatchingTest::compareMapMatching(int line,
                                           point::GeoPoint const &geoPoint,
                                           std::size_t const expectedNumberOfMatches)
{
  physics::Distance searchDist(1);

  struct TestResult
  {
    match::MapMatchedPosition mapMatchedPosition;
    lane::LaneDirection laneDirection;
  };

  std::vector<TestResult> testResults;

  mMapMatching->addRouteHint(mRouteHint);
  for (auto headingHint : mHeadingHints)
  {
    mMapMatching->addHeadingHint(headingHint, geoPoint);
  }

  auto mapMatchingResults = mMapMatching->getMapMatchedPositions(geoPoint, searchDist, mMinProbabilty);

  for (auto mapMatchingResult : mapMatchingResults)
  {
    TestResult result;
    result.mapMatchedPosition = mapMatchingResult;
    result.laneDirection = lane::getLane(result.mapMatchedPosition.lanePoint.paraPoint.laneId).direction;
    testResults.push_back(result);
  }

  ASSERT_EQ(expectedNumberOfMatches, testResults.size()) << " compareMapMatching called from " << line << "\n";

  if ((mRouteHint.roadSegments.size() > 0u) && (testResults.size() > 0u))
  {
    ASSERT_EQ(1u, mRouteHint.roadSegments.size()) << " compareMapMatching called from " << line << "\n";
    ASSERT_EQ(1u, mRouteHint.roadSegments[0].drivableLaneSegments.size()) << " compareMapMatching called from " << line
                                                                          << "\n";
    ASSERT_LT(0u, testResults.size()) << " compareMapMatching called from " << line << "\n";
    ASSERT_EQ(mRouteHint.roadSegments[0].drivableLaneSegments[0].laneInterval.laneId,
              testResults[0].mapMatchedPosition.lanePoint.paraPoint.laneId)
      << " compareMapMatching called from " << line << "\n";
  }
}

TEST_F(AdMapMatchingTest, perform_map_matching_no_route_hints)
{
  for (auto testElement : mTestPoints)
  {
    compareMapMatching(__LINE__, testElement.first, testElement.second);
  }
}

TEST_F(AdMapMatchingTest, isLanePartOfRouteHints)
{
  addRouteHint();

  mMapMatching->addRouteHint(mRouteHint);

  EXPECT_TRUE(mMapMatching->isLanePartOfRouteHints(
    mRouteHint.roadSegments.front().drivableLaneSegments.front().laneInterval.laneId));
  EXPECT_FALSE(mMapMatching->isLanePartOfRouteHints(
    mRouteHint.roadSegments.front().drivableLaneSegments.front().laneInterval.laneId + lane::LaneId(1)));
}

TEST_F(AdMapMatchingTest, perform_map_matching_with_route_hints)
{
  addRouteHint();
  for (auto testElement : mTestPoints)
  {
    compareMapMatching(__LINE__, testElement.first, testElement.second);
  }
}

TEST_F(AdMapMatchingTest, perform_map_matching_with_heading_hints)
{
  mMinProbabilty = physics::Probability(0.05);
  mHeadingHints.push_back(point::createENUHeading(point::degree2radians(70)));
  for (auto testElement : mTestPoints)
  {
    compareMapMatching(__LINE__, testElement.first, testElement.second);
  }

  mHeadingHints.clear();
  mHeadingHints.push_back(point::createENUHeading(point::degree2radians(-110)));
  for (auto testElement : mTestPoints)
  {
    compareMapMatching(__LINE__, testElement.first, testElement.second);
  }
}

TEST_F(AdMapMatchingTest, laneOperation)
{
  addRouteHint();

  mMapMatching->addRouteHint(mRouteHint);
  auto hintPoint = point::createGeoPoint(point::Longitude(8.4401803), point::Latitude(49.0191987), point::Altitude(0.));
  MapMatchedPositionConfidenceList mapMatchingResults;
  EXECUTE_NO_LOG(mapMatchingResults = mMapMatching->findLanes(point::ECEFPoint(), physics::Distance(0.01)));
  ASSERT_EQ(mapMatchingResults.size(), 0u);
  mapMatchingResults = mMapMatching->findLanes(toECEF(hintPoint), physics::Distance());
  ASSERT_EQ(mapMatchingResults.size(), 0u);
  mapMatchingResults = mMapMatching->findLanes(toECEF(hintPoint), physics::Distance(0.01));
  ASSERT_EQ(mapMatchingResults.size(), 1u);
  ASSERT_NEAR(double(ad::map::lane::calcWidth(toENU(hintPoint))), 3.0039, 0.0001);

  LaneOccupiedRegionList laneOccupiedRegions;
  LaneOccupiedRegionList otherLaneOccupiedRegions;

  LaneOccupiedRegion region1, region2;
  region1.laneId = map::lane::LaneId(10);
  physics::ParametricRange range;
  range.minimum = physics::ParametricValue(0.1);
  range.maximum = physics::ParametricValue(0.2);
  region1.longitudinalRange = range;
  range.maximum = physics::ParametricValue(0.3);
  region1.lateralRange = range;
  otherLaneOccupiedRegions.push_back(region1);
  mMapMatching->addLaneRegions(laneOccupiedRegions, otherLaneOccupiedRegions);
  ASSERT_EQ(laneOccupiedRegions[0].lateralRange, range);

  region2 = region1;
  range.maximum = physics::ParametricValue(0.4);
  region2.lateralRange = range;
  otherLaneOccupiedRegions.clear();
  otherLaneOccupiedRegions.push_back(region2);
  mMapMatching->addLaneRegions(laneOccupiedRegions, otherLaneOccupiedRegions);
  ASSERT_EQ(laneOccupiedRegions[0].lateralRange, range);
}
