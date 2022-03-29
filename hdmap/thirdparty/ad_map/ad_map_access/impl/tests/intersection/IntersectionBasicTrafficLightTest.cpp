// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/landmark/LandmarkOperation.hpp>
#include <ad/map/point/Operation.hpp>
#include "IntersectionTrafficLightTests.hpp"
#include "MapSetup.hpp"

namespace ad {
namespace map {

point::GeoPoint makeGeoFromNorth()
{
  return point::createGeoPoint(point::Longitude(8.43744), point::Latitude(49.01884), point::Altitude(0));
}

point::GeoPoint makeGeoToNorth()
{
  return point::createGeoPoint(point::Longitude(8.43748), point::Latitude(49.01878), point::Altitude(0));
}

point::GeoPoint makeGeoFromSouth()
{
  return point::createGeoPoint(point::Longitude(8.43748), point::Latitude(49.01852), point::Altitude(0));
}
point::GeoPoint makeGeoToSouth()
{
  return point::createGeoPoint(point::Longitude(8.43744), point::Latitude(49.01861), point::Altitude(0));
}

point::GeoPoint makeGeoFromWest()
{
  return point::createGeoPoint(point::Longitude(8.43717), point::Latitude(49.01866), point::Altitude(0));
}

point::GeoPoint makeGeoToWest()
{
  return point::createGeoPoint(point::Longitude(8.43737), point::Latitude(49.01869), point::Altitude(0));
}

point::GeoPoint makeGeoFromEast()
{
  return point::createGeoPoint(point::Longitude(8.43769), point::Latitude(49.01869), point::Altitude(0));
}

point::GeoPoint makeGeoToEast()
{
  return point::createGeoPoint(point::Longitude(8.43754), point::Latitude(49.01867), point::Altitude(0));
}

/* @todo rework base class
 * Ideally there is a base class (similar to IntersectionYieldTest) which uses the same coordinates
 * as are used in all the synthetic tests and only override prepareMap() to provide the map
 * SolidTrafficLights while all the points, e.g. fromNorth,  can be taken from
 * SyntheticIntersectionTestBase. But if we do so we loose all the existing functionality for
 * testing traffic light specific stuff as is implemented in IntersectionTrafficLightBase
 *
 * Therefore for all the test here, we always have the functions prepareMap plus all getGeoXXX with
 * the same content.
 */
struct IntersectionBasicTrafficLightSolidWestToSouthTest : IntersectionTrafficLightWestToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapSolidTrafficLights();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return makeGeoFromNorth();
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return makeGeoToNorth();
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return makeGeoFromSouth();
  }
  virtual point::GeoPoint getGeoToSouth() const override
  {
    return makeGeoToSouth();
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return makeGeoFromWest();
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return makeGeoToWest();
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return makeGeoFromEast();
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return makeGeoToEast();
  }
};

TEST_F(IntersectionBasicTrafficLightSolidWestToSouthTest, basic_checks)
{
  point::GeoPoint geoPoint;
  geoPoint = point::createGeoPoint(point::Longitude(8.4372417), point::Latitude(49.01864298), point::Altitude(3.));
  landmark::LandmarkId id;
  id = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks(
    {expectedTrafficLight((uint64_t)id, landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN)});
}

struct IntersectionBasicTrafficLightSolidWestToNorthTest : IntersectionTrafficLightWestToNorthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapSolidTrafficLights();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return makeGeoFromNorth();
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return makeGeoToNorth();
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return makeGeoFromSouth();
  }
  virtual point::GeoPoint getGeoToSouth() const override
  {
    return makeGeoToSouth();
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return makeGeoFromWest();
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return makeGeoToWest();
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return makeGeoFromEast();
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return makeGeoToEast();
  }

  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override
  {
    return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesSouth()});
  }

  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override
  {
    return createUnorderedLaneIdSet({getIncomingLanesEast()});
  }
};

TEST_F(IntersectionBasicTrafficLightSolidWestToNorthTest, basic_checks)
{
  point::GeoPoint geoPoint;
  geoPoint = point::createGeoPoint(point::Longitude(8.4372417), point::Latitude(49.01864298), point::Altitude(3.));
  landmark::LandmarkId id;
  id = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks(
    {expectedTrafficLight((uint64_t)id, landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN)});
}

struct IntersectionBasicTrafficLightSolidSouthToWestTest : IntersectionTrafficLightSouthToWestTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapSolidTrafficLights();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return makeGeoFromNorth();
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return makeGeoToNorth();
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return makeGeoFromSouth();
  }
  virtual point::GeoPoint getGeoToSouth() const override
  {
    return makeGeoToSouth();
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return makeGeoFromWest();
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return makeGeoToWest();
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return makeGeoFromEast();
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return makeGeoToEast();
  }

  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override
  {
    return createUnorderedLaneIdSet({getIncomingLanesEast(), getIncomingLanesWest()});
  }

  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override
  {
    return createUnorderedLaneIdSet({getIncomingLanesNorth()});
  }
};

TEST_F(IntersectionBasicTrafficLightSolidSouthToWestTest, basic_checks)
{
  point::GeoPoint geoPoint;
  geoPoint = point::createGeoPoint(point::Longitude(8.437514892), point::Latitude(49.01854345), point::Altitude(3.));
  landmark::LandmarkId id;
  id = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks(
    {expectedTrafficLight((uint64_t)id, landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN)});
}

struct IntersectionBasicTrafficLightSolidNorthToSouthTest : IntersectionTrafficLightNorthToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapSolidTrafficLights();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return makeGeoFromNorth();
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return makeGeoToNorth();
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return makeGeoFromSouth();
  }

  virtual point::GeoPoint getGeoToSouth() const override
  {
    return makeGeoToSouth();
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return makeGeoFromWest();
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return makeGeoToWest();
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return makeGeoFromEast();
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return makeGeoToEast();
  }
};

TEST_F(IntersectionBasicTrafficLightSolidNorthToSouthTest, basic_checks)
{
  point::GeoPoint geoPoint;
  geoPoint = point::createGeoPoint(point::Longitude(8.437399821), point::Latitude(49.0188076), point::Altitude(3.));
  landmark::LandmarkId id;
  id = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks(
    {expectedTrafficLight((uint64_t)id, landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN)});
}

} // namespace map
} // namespace ad
