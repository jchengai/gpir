// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2019-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/access/Operation.hpp>
#include <ad/map/intersection/Intersection.hpp>
#include <ad/map/match/Operation.hpp>
#include <ad/map/route/Planning.hpp>
#include <ad/map/route/RoutePrediction.hpp>
#include <gtest/gtest.h>

using namespace ::ad;
using namespace map;
using namespace map::point;
using namespace map::route;
using namespace map::route::planning;

struct RoutePredictionTest : ::testing::Test
{
  RoutePredictionTest()
  {
  }

  virtual void SetUp()
  {
    access::cleanup();
    if (!access::init(getTestMap()))
    {
      throw std::runtime_error("Unable to initialize with " + getTestMap());
    }

    auto predictionStartGeo = getPredictionStartGeo();
    ASSERT_TRUE(withinValidInputRange(predictionStartGeo));
    match::AdMapMatching mapMatching;
    auto mapMatchingResults
      = mapMatching.getMapMatchedPositions(predictionStartGeo, physics::Distance(1.), physics::Probability(0.8));

    ASSERT_GE(mapMatchingResults.size(), 1u);
    predictionStart.point = mapMatchingResults.front().lanePoint.paraPoint;
    if (lane::isLaneDirectionPositive(lane::getLane(predictionStart.point.laneId)))
    {
      predictionStart.direction = route::planning::RoutingDirection::POSITIVE;
    }
    else
    {
      predictionStart.direction = route::planning::RoutingDirection::NEGATIVE;
    }
  }

  virtual void TearDown()
  {
    access::cleanup();
  }

  virtual std::string getTestMap() = 0;

  virtual point::GeoPoint getPredictionStartGeo()
  {
    point::GeoPoint resultPoint;
    auto pois = access::getPointsOfInterest();
    if (pois.size() > 0u)
    {
      resultPoint = pois.front().geoPoint;
    }
    return resultPoint;
  }

  route::planning::RoutingParaPoint predictionStart;
};

struct RoutePredictionTestTown01 : public RoutePredictionTest
{
  std::string getTestMap() override
  {
    return "test_files/Town01.txt";
  }
};

struct RoutePredictionTestTown03 : public RoutePredictionTest
{
  std::string getTestMap() override
  {
    return "test_files/Town03.txt";
  }
};

TEST_F(RoutePredictionTestTown01, route_prediction_positive)
{
  auto routePredictions = route::planning::predictRoutesOnDistance(predictionStart, physics::Distance(10.));
  EXPECT_EQ(routePredictions.size(), 1u);

  routePredictions = route::planning::predictRoutesOnDistance(predictionStart, physics::Distance(450.));
  EXPECT_EQ(routePredictions.size(), 6u);
}

TEST_F(RoutePredictionTestTown01, route_prediction_dont_care)
{
  predictionStart.direction = route::planning::RoutingDirection::DONT_CARE;
  auto routePredictions = route::planning::predictRoutesOnDistance(predictionStart, physics::Distance(10.));
  EXPECT_EQ(routePredictions.size(), 1u);

  routePredictions = route::planning::predictRoutesOnDistance(predictionStart, physics::Distance(450.));
  EXPECT_EQ(routePredictions.size(), 6u);
}

TEST_F(RoutePredictionTestTown03, route_prediction_positive)
{
  auto routePredictions = route::planning::predictRoutesOnDistance(predictionStart, physics::Distance(10.));
  EXPECT_EQ(routePredictions.size(), 1u);

  routePredictions = route::planning::predictRoutesOnDistance(predictionStart, physics::Distance(478.));
  EXPECT_EQ(routePredictions.size(), 18u);
}

TEST_F(RoutePredictionTestTown03, route_prediction_dont_care)
{
  predictionStart.direction = route::planning::RoutingDirection::DONT_CARE;
  auto routePredictions = route::planning::predictRoutesOnDistance(predictionStart, physics::Distance(10.));
  EXPECT_EQ(routePredictions.size(), 1u);

  routePredictions = route::planning::predictRoutesOnDistance(predictionStart, physics::Distance(478.));
  EXPECT_EQ(routePredictions.size(), 18u);
}

TEST_F(RoutePredictionTestTown03, route_prediction_constructor)
{
  predictionStart.direction = route::planning::RoutingDirection::DONT_CARE;
  auto routePredictions = route::planning::predictRoutesOnDuration(predictionStart, physics::Duration(1.));
  ASSERT_EQ(routePredictions.size(), 1u);

  routePredictions = route::planning::predictRoutes(predictionStart, physics::Distance(478.), physics::Duration(478.));
  ASSERT_EQ(routePredictions.size(), 18u);
}

TEST_F(RoutePredictionTestTown03, route_getBasicRoutes)
{
  predictionStart.direction = route::planning::RoutingDirection::DONT_CARE;
  map::route::planning::RoutePrediction routePrediction(predictionStart, physics::Distance(10.));
  if (routePrediction.calculate())
  {
    std::vector<Route::BasicRoute> res = routePrediction.getBasicRoutes();
  }
}

struct RoutePredictionTestIntersection : public RoutePredictionTestTown01
{
  point::GeoPoint getPredictionStartGeo() override
  {
    return point::createGeoPoint(point::Longitude(0.00079520), point::Latitude(-0.00284188), point::Altitude(0.));
  }
};

TEST_F(RoutePredictionTestIntersection, route_prediction_dont_stop_within_intersections)
{
  auto routePredictions = route::planning::predictRoutesOnDistance(predictionStart, physics::Distance(7.));
  EXPECT_EQ(routePredictions.size(), 2u);

  for (auto &routePrediction : routePredictions)
  {
    auto intersections = intersection::Intersection::getIntersectionsForRoute(routePrediction);
    ASSERT_EQ(1u, intersections.size());
  }
}

TEST_F(RoutePredictionTestIntersection, route_extension_dont_stop_within_intersections)
{
  auto routePredictions = route::planning::predictRoutesOnDistance(predictionStart, physics::Distance(1.));
  EXPECT_EQ(routePredictions.size(), 1u);

  route::FullRoute route = routePredictions.front();
  routePredictions.clear();

  EXPECT_TRUE(route::extendRouteToDistance(route, physics::Distance(7.), routePredictions));

  routePredictions.push_back(route);

  for (auto &routePrediction : routePredictions)
  {
    auto intersections = intersection::Intersection::getIntersectionsForRoute(routePrediction);
    ASSERT_EQ(1u, intersections.size());
  }
}

struct RoutePredictionTestRoundabout : public RoutePredictionTestTown03
{
  point::GeoPoint getPredictionStartGeo() override
  {
    return point::createGeoPoint(point::Longitude(0.00033117), point::Latitude(0.00007233), point::Altitude(0.));
  }
};

TEST_F(RoutePredictionTestRoundabout, route_prediction)
{
  auto routePredictions = route::planning::predictRoutesOnDistance(predictionStart, physics::Distance(130.));
  // starting at the east entry we get:
  // - to north
  // - to south
  // - circling just before out to east
  // the route towards the west is not present in the new map!
  EXPECT_EQ(routePredictions.size(), 3u);
}
