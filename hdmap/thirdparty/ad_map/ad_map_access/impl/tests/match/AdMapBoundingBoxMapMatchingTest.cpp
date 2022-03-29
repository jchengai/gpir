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
#include <ad/map/point/Operation.hpp>
#include <algorithm>
#include <gtest/gtest.h>
#include "ad/map/match/MapMatchedOperation.hpp"

using namespace ::ad;
using namespace ::ad::map;
using namespace ::ad::map::match;

struct AdMapBoundingBoxMapMatchingTest : ::testing::Test
{
  AdMapBoundingBoxMapMatchingTest()
  {
  }

  virtual void SetUp()
  {
    access::cleanup();
    access::init("test_files/TPK.adm.txt");

    mMinProbabilty = physics::Probability(0.05);
    mSamplingDistance = physics::Distance(1.);

    config::PointOfInterest poi;
    ASSERT_TRUE(access::getPointOfInterest("T1", poi));
    mObjectPosition.enuReferencePoint = access::getENUReferencePoint();
    mObjectPosition.centerPoint = point::toENU(poi.geoPoint);
    mObjectPosition.heading = point::createENUHeading(M_PI_2);
    mObjectPosition.dimension.width = physics::Distance(0.7);
    mObjectPosition.dimension.length = physics::Distance(4.);
    mObjectPosition.dimension.height = physics::Distance(0.);
  }

  virtual void TearDown()
  {
    // make sure that we leave the singleton "clean"
    access::cleanup();
  }

  physics::Probability mMinProbabilty;
  physics::Distance mSamplingDistance;
  match::ENUObjectPosition mObjectPosition;
};

TEST_F(AdMapBoundingBoxMapMatchingTest, box_within_single_lane)
{
  match::AdMapMatching mapMatching;
  auto centerMapMatched = mapMatching.getMapMatchedPositions(mObjectPosition, mSamplingDistance, mMinProbabilty);

  point::ParaPointList para = getParaPoints(centerMapMatched);
  ASSERT_EQ(para.size(), 2u);

  ASSERT_FALSE(centerMapMatched.empty());
  auto heading = mapMatching.getLaneENUHeading(centerMapMatched.front());

  mObjectPosition.heading = heading;
  mObjectPosition.dimension.width = physics::Distance(1.5);
  mObjectPosition.dimension.length = physics::Distance(3);

  auto result = mapMatching.getMapMatchedBoundingBox(mObjectPosition, mSamplingDistance);

  ASSERT_EQ(result.laneOccupiedRegions.size(), 1u);
  for (auto centerMatch : centerMapMatched)
  {
    if (centerMatch.type == match::MapMatchedPositionType::LANE_IN)
    {
      ASSERT_EQ(centerMatch.lanePoint.paraPoint.laneId, result.laneOccupiedRegions.front().laneId);
      ASSERT_LE(result.laneOccupiedRegions.front().lateralRange.minimum,
                result.laneOccupiedRegions.front().lateralRange.maximum);
      ASSERT_LE(result.laneOccupiedRegions.front().longitudinalRange.minimum,
                result.laneOccupiedRegions.front().longitudinalRange.maximum);
    }
  }

  ASSERT_EQ(getObjectENUHeading(result), heading);
  match::MapMatchedObjectBoundingBox saveResult = result;
  result.referencePointPositions[int32_t(match::ObjectReferencePoints::RearLeft)].clear();
  result.referencePointPositions[int32_t(match::ObjectReferencePoints::RearRight)].clear();
  result.referencePointPositions[int32_t(match::ObjectReferencePoints::FrontLeft)].clear();
  result.referencePointPositions[int32_t(match::ObjectReferencePoints::FrontRight)].clear();
  EXPECT_THROW(getObjectENUHeading(result), std::runtime_error);
  result = saveResult;
  result.referencePointPositions[int32_t(match::ObjectReferencePoints::RearRight)].clear();
  result.referencePointPositions[int32_t(match::ObjectReferencePoints::FrontRight)].clear();
  ASSERT_EQ(getObjectENUHeading(result), heading);
  result = saveResult;
  result.referencePointPositions[int32_t(match::ObjectReferencePoints::RearLeft)].clear();
  result.referencePointPositions[int32_t(match::ObjectReferencePoints::FrontLeft)].clear();
  ASSERT_EQ(getObjectENUHeading(result), heading);
  result = saveResult;
  result.referencePointPositions[int32_t(match::ObjectReferencePoints::RearLeft)].clear();
  result.referencePointPositions[int32_t(match::ObjectReferencePoints::RearRight)].clear();
  ASSERT_EQ(getObjectENUHeading(result), heading);
  result = saveResult;
  result.referencePointPositions[int32_t(match::ObjectReferencePoints::FrontLeft)].clear();
  result.referencePointPositions[int32_t(match::ObjectReferencePoints::FrontRight)].clear();
  ASSERT_EQ(getObjectENUHeading(result), heading);
  result = saveResult;

  std::vector<ENUObjectPosition> enuObjs;
  enuObjs.push_back(mObjectPosition);
  LaneOccupiedRegionList occRegion = mapMatching.getLaneOccupiedRegions(enuObjs, mSamplingDistance);

  lane::LaneId x11;
  ASSERT_EQ(occRegion.size(), 1u);
  for (auto centerMatch : centerMapMatched)
  {
    if (centerMatch.type == match::MapMatchedPositionType::LANE_IN)
    {
      x11 = centerMatch.lanePoint.paraPoint.laneId;
      ASSERT_EQ(centerMatch.lanePoint.paraPoint.laneId, occRegion.front().laneId);
    }
  }

  physics::Distance dis = signedDistanceToLane(occRegion.front().laneId, centerMapMatched);
  ASSERT_EQ(dis, physics::Distance(0));

  lane::LaneId x(100);
  MapMatchedPositionConfidenceList matchList;
  MapMatchedPosition matchPos;
  matchPos.lanePoint.laneWidth = physics::Distance(1.0);
  matchPos.lanePoint.paraPoint.laneId = x;

  matchPos.type = match::MapMatchedPositionType::LANE_LEFT;
  matchPos.lanePoint.lateralT = physics::RatioValue(1.0);
  matchList.clear();
  matchList.push_back(matchPos);
  EXPECT_THROW(signedDistanceToLane(x, matchList), std::runtime_error);

  matchPos.type = match::MapMatchedPositionType::LANE_LEFT;
  matchPos.lanePoint.lateralT = physics::RatioValue(-1.0);
  matchList.clear();
  matchList.push_back(matchPos);
  ASSERT_EQ(signedDistanceToLane(x, matchList), physics::Distance(-1.0));

  matchPos.type = match::MapMatchedPositionType::LANE_RIGHT;
  matchPos.lanePoint.lateralT = physics::RatioValue(0.8);
  matchList.clear();
  matchList.push_back(matchPos);
  EXPECT_THROW(signedDistanceToLane(x, matchList), std::runtime_error);

  matchPos.type = match::MapMatchedPositionType::LANE_RIGHT;
  matchPos.lanePoint.lateralT = physics::RatioValue(1.5);
  matchList.clear();
  matchList.push_back(matchPos);
  ASSERT_EQ(signedDistanceToLane(x, matchList), physics::Distance(0.5));

  matchPos.type = match::MapMatchedPositionType::INVALID;
  matchPos.lanePoint.lateralT = physics::RatioValue(1.0);
  matchList.clear();
  matchList.push_back(matchPos);
  EXPECT_THROW(signedDistanceToLane(x, matchList), std::runtime_error);

  Object obj;
  obj.enuPosition = mObjectPosition;
  obj.mapMatchedBoundingBox = result;
  dis = getDistanceToLane(x11, obj);
  ASSERT_EQ(dis, physics::Distance(0));
  dis = getDistanceToLane(para[0].laneId, obj);
  ASSERT_NEAR((double)dis, 1.7429, 0.0001);
}

TEST_F(AdMapBoundingBoxMapMatchingTest, rotated_box_within_two_lateral_lanes)
{
  match::AdMapMatching mapMatching;
  auto centerMapMatched = mapMatching.getMapMatchedPositions(mObjectPosition, mSamplingDistance, mMinProbabilty);

  ASSERT_EQ(centerMapMatched.size(), 2u);
  auto heading = mapMatching.getLaneENUHeading(centerMapMatched.front());

  mObjectPosition.heading = heading;
  mObjectPosition.dimension.width = physics::Distance(4.);
  mObjectPosition.dimension.length = physics::Distance(1.5);

  auto result = mapMatching.getMapMatchedBoundingBox(mObjectPosition, mSamplingDistance);

  ASSERT_EQ(result.laneOccupiedRegions.size(), 2u);

  auto searchFront = std::find_if(result.laneOccupiedRegions.begin(),
                                  result.laneOccupiedRegions.end(),
                                  [&centerMapMatched](match::LaneOccupiedRegion const &other) {
                                    return other.laneId == centerMapMatched.front().lanePoint.paraPoint.laneId;
                                  });
  ASSERT_TRUE(searchFront != std::end(result.laneOccupiedRegions));

  auto searchBack = std::find_if(result.laneOccupiedRegions.begin(),
                                 result.laneOccupiedRegions.end(),
                                 [&centerMapMatched](match::LaneOccupiedRegion const &other) {
                                   return other.laneId == centerMapMatched.back().lanePoint.paraPoint.laneId;
                                 });
  ASSERT_TRUE(searchBack != std::end(result.laneOccupiedRegions));

  // ensure the whole lane region up the the lane borders is covered
  auto laneContactRelation = lane::getDirectNeighborhoodRelation(centerMapMatched.front().lanePoint.paraPoint.laneId,
                                                                 centerMapMatched.back().lanePoint.paraPoint.laneId);
  if (laneContactRelation == lane::ContactLocation::LEFT)
  {
    ASSERT_EQ(physics::ParametricValue(0.), searchFront->lateralRange.minimum);
    ASSERT_LT(physics::ParametricValue(0.), searchFront->lateralRange.maximum);
    ASSERT_EQ(physics::ParametricValue(1.), searchBack->lateralRange.maximum);
    ASSERT_GT(physics::ParametricValue(1.), searchFront->lateralRange.minimum);
  }
  else if (laneContactRelation == lane::ContactLocation::RIGHT)
  {
    ASSERT_EQ(physics::ParametricValue(0.), searchBack->lateralRange.minimum);
    ASSERT_LT(physics::ParametricValue(0.), searchBack->lateralRange.maximum);
    ASSERT_EQ(physics::ParametricValue(1.), searchFront->lateralRange.maximum);
    ASSERT_LT(physics::ParametricValue(1.), searchFront->lateralRange.minimum);
  }
  else
  {
    ASSERT_TRUE(false);
  }

  // reconstruct length and width of vehicle
  physics::Distance vehicleWidth(0.);
  for (auto const &occupiedRegion : result.laneOccupiedRegions)
  {
    auto const vehicleLength = lane::getLane(occupiedRegion.laneId).length
      * (occupiedRegion.longitudinalRange.maximum - occupiedRegion.longitudinalRange.minimum);
    ASSERT_LE(vehicleLength * 0.9, mObjectPosition.dimension.length);
    ASSERT_GE(vehicleLength * 1.1, mObjectPosition.dimension.length);

    vehicleWidth += lane::getLane(occupiedRegion.laneId).width
      * (occupiedRegion.lateralRange.maximum - occupiedRegion.lateralRange.minimum);
  }

  ASSERT_LE(vehicleWidth * 0.9, mObjectPosition.dimension.width);
  ASSERT_GE(vehicleWidth * 1.1, mObjectPosition.dimension.width);
}

TEST_F(AdMapBoundingBoxMapMatchingTest, box_covering_three_lanes_longitudinal)
{
  auto objectCenter = point::toENU(
    point::createGeoPoint(point::Longitude(8.439497056), point::Latitude(49.018312553), point::Altitude(0.)));

  match::AdMapMatching mapMatching;
  auto centerMapMatched = mapMatching.getMapMatchedPositions(objectCenter, mSamplingDistance, mMinProbabilty);

  ASSERT_EQ(centerMapMatched.size(), 1u);

  auto const centerLaneId = centerMapMatched.front().lanePoint.paraPoint.laneId;
  auto objectPosition = mObjectPosition;
  auto heading = mapMatching.getLaneENUHeading(centerMapMatched.front());

  objectPosition.heading = point::createENUHeading(static_cast<double>(heading));
  objectPosition.centerPoint = objectCenter;
  objectPosition.dimension.width = physics::Distance(2.5);
  objectPosition.dimension.length = physics::Distance(6.);

  auto result = mapMatching.getMapMatchedBoundingBox(objectPosition, mSamplingDistance);

  ASSERT_EQ(result.laneOccupiedRegions.size(), 3u);

  // reconstruct length and width of vehicle
  physics::Distance vehicleLength(0.);
  for (auto const &occupiedRegion : result.laneOccupiedRegions)
  {
    // longitudinal
    if (occupiedRegion.laneId == centerLaneId)
    {
      // center lane fully covered
      ASSERT_EQ(physics::ParametricValue(0.), occupiedRegion.longitudinalRange.minimum);
      ASSERT_EQ(physics::ParametricValue(1.), occupiedRegion.longitudinalRange.maximum);
    }
    else if (occupiedRegion.longitudinalRange.minimum == physics::ParametricValue(0.))
    {
      // other lane not fully covered, but definitely some parts of it
      ASSERT_NE(physics::ParametricValue(0.), occupiedRegion.longitudinalRange.maximum);
      ASSERT_NE(physics::ParametricValue(1.), occupiedRegion.longitudinalRange.maximum);
    }
    else
    {
      // other lane not fully covered, but definitely some parts of it
      ASSERT_EQ(physics::ParametricValue(1.), occupiedRegion.longitudinalRange.maximum);
      ASSERT_NE(physics::ParametricValue(0.), occupiedRegion.longitudinalRange.minimum);
      ASSERT_NE(physics::ParametricValue(1.), occupiedRegion.longitudinalRange.minimum);
    }

    vehicleLength += lane::getLane(occupiedRegion.laneId).length
      * (occupiedRegion.longitudinalRange.maximum - occupiedRegion.longitudinalRange.minimum);

    // lateral always in between
    ASSERT_NE(physics::ParametricValue(0.), occupiedRegion.lateralRange.minimum);
    ASSERT_NE(physics::ParametricValue(1.), occupiedRegion.lateralRange.minimum);
    ASSERT_NE(physics::ParametricValue(0.), occupiedRegion.lateralRange.maximum);
    ASSERT_NE(physics::ParametricValue(1.), occupiedRegion.lateralRange.maximum);

    auto const vehicleWidth = lane::getLane(occupiedRegion.laneId).width
      * (occupiedRegion.lateralRange.maximum - occupiedRegion.lateralRange.minimum);
    ASSERT_LE(vehicleWidth * 0.9, objectPosition.dimension.width);
    ASSERT_GE(vehicleWidth * 1.1, objectPosition.dimension.width);
  }

  ASSERT_LE(vehicleLength * 0.9, objectPosition.dimension.length);
  ASSERT_GE(vehicleLength * 1.1, objectPosition.dimension.length);
}

TEST_F(AdMapBoundingBoxMapMatchingTest, box_not_touching_second_lane_within_sampling_distance)
{
  auto objectCenter = point::toENU(
    point::createGeoPoint(point::Longitude(8.4394653), point::Latitude(49.0182735), point::Altitude(0.)));

  match::AdMapMatching mapMatching;
  auto centerMapMatched = mapMatching.getMapMatchedPositions(objectCenter, mSamplingDistance, mMinProbabilty);

  ASSERT_EQ(centerMapMatched.size(), 1u);

  auto objectPosition = mObjectPosition;
  objectPosition.heading = mapMatching.getLaneENUHeading(centerMapMatched.front());
  objectPosition.centerPoint = objectCenter;
  objectPosition.dimension.width = physics::Distance(2.5);
  objectPosition.dimension.length = physics::Distance(6.);

  auto result = mapMatching.getMapMatchedBoundingBox(objectPosition, mSamplingDistance);

  ASSERT_EQ(result.laneOccupiedRegions.size(), 1u);
}
