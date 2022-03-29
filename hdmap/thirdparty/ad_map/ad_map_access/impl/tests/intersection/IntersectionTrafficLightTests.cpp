// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "IntersectionTrafficLightTests.hpp"
#include <ad/map/landmark/LandmarkOperation.hpp>
#include <ad/map/point/Operation.hpp>
#include "IntersectionTown01Test.hpp"
#include "MapSetup.hpp"

namespace ad {
namespace map {

// @todo rework required
// Always assumes that lanes with higher priority are always empty. This is not true if there
// is a solid traffic light and the host vehicle is turning left.
// Need to check expectedIncomingLanesWithHigherPriority and expectedIncomingLanesWithLowerPriority

void IntersectionTrafficLightTest::prepareMap() const
{
  ::map_setup::prepareMapTrafficLightsPfz();
}

lane::LaneId IntersectionTrafficLightFromWestTest::getRouteStart() const
{
  return mFromWest;
}

intersection::IntersectionType IntersectionTrafficLightFromWestTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::TrafficLight;
}

lane::LaneId IntersectionTrafficLightWestToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionTrafficLightWestToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightWestToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesEast(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionTrafficLightWestToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightWestToNorthTest::expectedCrossingLanes() const
{
  return lane::LaneIdSet();
}

TEST_F(IntersectionTrafficLightWestToNorthTest, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1, id2;
  geoPoint = point::createGeoPoint(point::Longitude(8.457537128), point::Latitude(49.02076074), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.457463354), point::Latitude(49.02067458), point::Altitude(3.));
  id2 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks({expectedTrafficLight((uint64_t)id1), expectedTrafficLight((uint64_t)id2)});
}

lane::LaneId IntersectionTrafficLightWestToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionTrafficLightWestToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightWestToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesEast(), getIncomingLanesNorth(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionTrafficLightWestToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightWestToEastTest::expectedCrossingLanes() const
{
  return lane::LaneIdSet();
}

TEST_F(IntersectionTrafficLightWestToEastTest, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1, id2;
  geoPoint = point::createGeoPoint(point::Longitude(8.457537128), point::Latitude(49.02076074), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.457463354), point::Latitude(49.02067458), point::Altitude(3.));
  id2 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks({expectedTrafficLight((uint64_t)id2), expectedTrafficLight((uint64_t)id1)});
}

lane::LaneId IntersectionTrafficLightWestToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionTrafficLightWestToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightWestToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesEast(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionTrafficLightWestToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightWestToSouthTest::expectedCrossingLanes() const
{
  return lane::LaneIdSet();
}
TEST_F(IntersectionTrafficLightWestToSouthTest, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1;
  geoPoint = point::createGeoPoint(point::Longitude(8.457462013), point::Latitude(49.02067301), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks({expectedTrafficLight((uint64_t)id1)});
}

lane::LaneId IntersectionTrafficLightFromEastTest::getRouteStart() const
{
  return mFromEast;
}

intersection::IntersectionType IntersectionTrafficLightFromEastTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::TrafficLight;
}

lane::LaneId IntersectionTrafficLightEastToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionTrafficLightEastToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightEastToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesWest(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionTrafficLightEastToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightEastToNorthTest::expectedCrossingLanes() const
{
  return lane::LaneIdSet();
}

TEST_F(IntersectionTrafficLightEastToNorthTest, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1, id2, id3;
  geoPoint = point::createGeoPoint(point::Longitude(8.457971538), point::Latitude(49.0205365), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.458004075), point::Latitude(49.02058106), point::Altitude(5.));
  id2 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.458036612), point::Latitude(49.02062562), point::Altitude(3.));
  id3 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks(
    {expectedTrafficLight((uint64_t)id1), expectedTrafficLight((uint64_t)id2), expectedTrafficLight((uint64_t)id3)});
}

lane::LaneId IntersectionTrafficLightEastToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionTrafficLightEastToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightEastToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesWest(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionTrafficLightEastToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightEastToWestTest::expectedCrossingLanes() const
{
  return lane::LaneIdSet();
}

TEST_F(IntersectionTrafficLightEastToWestTest, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1, id2, id3;
  geoPoint = point::createGeoPoint(point::Longitude(8.457971538), point::Latitude(49.0205365), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.458004075), point::Latitude(49.02058106), point::Altitude(5.));
  id2 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.458036612), point::Latitude(49.02062562), point::Altitude(3.));
  id3 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks(
    {expectedTrafficLight((uint64_t)id1), expectedTrafficLight((uint64_t)id2), expectedTrafficLight((uint64_t)id3)});
}

lane::LaneId IntersectionTrafficLightEastToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionTrafficLightEastToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionTrafficLightEastToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionTrafficLightEastToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightEastToSouthTest::expectedCrossingLanes() const
{
  return lane::LaneIdSet();
}

TEST_F(IntersectionTrafficLightEastToSouthTest, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1, id2, id3;
  geoPoint = point::createGeoPoint(point::Longitude(8.457971538), point::Latitude(49.0205365), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.458004075), point::Latitude(49.02058106), point::Altitude(5.));
  id2 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.458036612), point::Latitude(49.02062562), point::Altitude(3.));
  id3 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks(
    {expectedTrafficLight((uint64_t)id1), expectedTrafficLight((uint64_t)id2), expectedTrafficLight((uint64_t)id3)});
}

lane::LaneId IntersectionTrafficLightFromNorthTest::getRouteStart() const
{
  return mFromNorth;
}

intersection::IntersectionType IntersectionTrafficLightFromNorthTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::TrafficLight;
}

lane::LaneId IntersectionTrafficLightNorthToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionTrafficLightNorthToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionTrafficLightNorthToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionTrafficLightNorthToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightNorthToEastTest::expectedCrossingLanes() const
{
  return lane::LaneIdSet();
}

TEST_F(IntersectionTrafficLightNorthToEastTest, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1;
  geoPoint = point::createGeoPoint(point::Longitude(8.457771924), point::Latitude(49.02077962), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks({expectedTrafficLight((uint64_t)id1)});
}

lane::LaneId IntersectionTrafficLightNorthToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionTrafficLightNorthToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightNorthToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesWest(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionTrafficLightNorthToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightNorthToWestTest::expectedCrossingLanes() const
{
  return lane::LaneIdSet();
}

TEST_F(IntersectionTrafficLightNorthToWestTest, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1;
  geoPoint = point::createGeoPoint(point::Longitude(8.457771924), point::Latitude(49.02077962), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks({expectedTrafficLight((uint64_t)id1)});
}

lane::LaneId IntersectionTrafficLightNorthToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionTrafficLightNorthToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightNorthToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesWest(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionTrafficLightNorthToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightNorthToSouthTest::expectedCrossingLanes() const
{
  return lane::LaneIdSet();
}

TEST_F(IntersectionTrafficLightNorthToSouthTest, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1;
  geoPoint = point::createGeoPoint(point::Longitude(8.457771924), point::Latitude(49.02077962), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks({expectedTrafficLight((uint64_t)id1)});
}

lane::LaneId IntersectionTrafficLightFromSouthTest::getRouteStart() const
{
  return mFromSouth;
}

intersection::IntersectionType IntersectionTrafficLightFromSouthTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::TrafficLight;
}

lane::LaneId IntersectionTrafficLightSouthToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionTrafficLightSouthToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightSouthToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesWest(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionTrafficLightSouthToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightSouthToEastTest::expectedCrossingLanes() const
{
  return lane::LaneIdSet();
}

TEST_F(IntersectionTrafficLightSouthToEastTest, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1, id2;
  geoPoint = point::createGeoPoint(point::Longitude(8.457704425), point::Latitude(49.02050956), point::Altitude(5.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.457737438), point::Latitude(49.02050359), point::Altitude(3.));
  id2 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks({expectedTrafficLight((uint64_t)id1), expectedTrafficLight((uint64_t)id2)});
}

lane::LaneId IntersectionTrafficLightSouthToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionTrafficLightSouthToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightSouthToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesWest(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionTrafficLightSouthToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

TEST_F(IntersectionTrafficLightSouthToWestTest, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1, id2;
  geoPoint = point::createGeoPoint(point::Longitude(8.457552564), point::Latitude(49.02053703), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.457664809), point::Latitude(49.02051672), point::Altitude(5.));
  id2 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks({expectedTrafficLight((uint64_t)id1), expectedTrafficLight((uint64_t)id2)});
}

lane::LaneIdSet IntersectionTrafficLightSouthToWestTest::expectedCrossingLanes() const
{
  return lane::LaneIdSet();
}

lane::LaneId IntersectionTrafficLightSouthToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionTrafficLightSouthToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightSouthToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesWest(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionTrafficLightSouthToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionTrafficLightSouthToNorthTest::expectedCrossingLanes() const
{
  return lane::LaneIdSet();
}

TEST_F(IntersectionTrafficLightSouthToNorthTest, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1, id2;
  geoPoint = point::createGeoPoint(point::Longitude(8.457552564), point::Latitude(49.02053703), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.457664809), point::Latitude(49.02051672), point::Altitude(5.));
  id2 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks({expectedTrafficLight((uint64_t)id1), expectedTrafficLight((uint64_t)id2)});
}

struct IntersectionTrafficLightTown01WestToSouthTest : IntersectionTown01WestToSouthTest,
                                                       IntersectionTrafficLightWestToSouthTest
{
  point::GeoPoint getGeoFromSouth() const override
  {
    return IntersectionTown01WestToSouthTest::getGeoFromSouth();
  }

  point::GeoPoint getGeoToSouth() const override
  {
    return IntersectionTown01WestToSouthTest::getGeoToSouth();
  }

  point::GeoPoint getGeoFromWest() const override
  {
    return IntersectionTown01WestToSouthTest::getGeoFromWest();
  }

  point::GeoPoint getGeoToWest() const override
  {
    return IntersectionTown01WestToSouthTest::getGeoToWest();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return IntersectionTown01WestToSouthTest::getGeoFromNorth();
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return IntersectionTown01WestToSouthTest::getGeoToNorth();
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return IntersectionTown01WestToSouthTest::getGeoFromEast();
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return IntersectionTown01WestToSouthTest::getGeoToEast();
  }

  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTown01TrafficLight();
  }
};

TEST_F(IntersectionTrafficLightTown01WestToSouthTest, basic_checks)
{
  IntersectionTown01WestToSouthTest::performBasicChecks();
}

} // namespace map
} // namespace ad
