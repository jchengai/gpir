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

struct IntersectionTrafficLightHirtenwegToHaid : IntersectionTrafficLightWestToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.440391), point::Latitude(49.017906), point::Altitude(0)); // Haid-und-Neu-Strasse
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return point::GeoPoint();
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return point::GeoPoint();
  }
  virtual point::GeoPoint getGeoToSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.439804), point::Latitude(49.017619), point::Altitude(0)); // Haid-und-Neu-Strasse
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.440130), point::Latitude(49.018100), point::Altitude(0)); // Hirtenweg
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::GeoPoint();
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return point::GeoPoint();
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return point::GeoPoint();
  }
};

TEST_F(IntersectionTrafficLightHirtenwegToHaid, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1, id2;
  geoPoint = point::createGeoPoint(point::Longitude(8.440227899), point::Latitude(49.01807585), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.440173867), point::Latitude(49.01804824), point::Altitude(3.));
  id2 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks({expectedTrafficLight((uint64_t)id2), expectedTrafficLight((uint64_t)id1)});
}

struct IntersectionTrafficLightHaidCrosswalk : IntersectionTrafficLightNorthToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.438906), point::Latitude(49.017112), point::Altitude(0)); // Haid-und-Neu-Strasse
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return point::GeoPoint();
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return point::GeoPoint();
  }
  virtual point::GeoPoint getGeoToSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4385758), point::Latitude(49.0169274), point::Altitude(0)); // Haid-und-Neu-Strasse
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::GeoPoint();
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::GeoPoint();
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return point::GeoPoint();
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return point::GeoPoint();
  }
};

TEST_F(IntersectionTrafficLightHaidCrosswalk, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1, id2;
  geoPoint = point::createGeoPoint(point::Longitude(8.438836635), point::Latitude(49.01700919), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.438765768), point::Latitude(49.01706402), point::Altitude(3.));
  id2 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks({expectedTrafficLight((uint64_t)id1), expectedTrafficLight((uint64_t)id2)});
}

struct IntersectionTrafficLightHaidToOstring : IntersectionTrafficLightNorthToEastTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.436480), point::Latitude(49.015874), point::Altitude(0.)); // Haid-und-Neu-Strasse
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.436519), point::Latitude(49.015797), point::Altitude(0.)); // Haid-und-Neu-Strasse
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.435849), point::Latitude(49.015530), point::Altitude(0.)); // Haid-und-Neu-Strasse
  }
  virtual point::GeoPoint getGeoToSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.435822), point::Latitude(49.015617), point::Altitude(0.)); // Haid-und-Neu-Strasse
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::GeoPoint();
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::GeoPoint();
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.436399), point::Latitude(49.015511), point::Altitude(0.)); // Ostring
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.436145), point::Latitude(49.015490), point::Altitude(0.)); // Ostring
  }

  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override
  {
    return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesEast()});
  }

  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override
  {
    return lane::LaneIdSet();
  }
};

TEST_F(IntersectionTrafficLightHaidToOstring, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1;
  geoPoint = point::createGeoPoint(point::Longitude(8.436353022), point::Latitude(49.01579163), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks({expectedTrafficLight((uint64_t)id1)});
}

struct IntersectionTrafficLightElfNordtangente : IntersectionTrafficLightSouthToNorthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45743), point::Latitude(49.01878), point::Altitude(0)); // Elfmorgenbruchstrasse
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45748), point::Latitude(49.01868), point::Altitude(0)); // Elfmorgenbruchstrasse
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45750), point::Latitude(49.01825), point::Altitude(0)); // Elfmorgenbruchstrasse
  }
  virtual point::GeoPoint getGeoToSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45743), point::Latitude(49.01844), point::Altitude(0)); // Elfmorgenbruchstrasse
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::GeoPoint();
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::GeoPoint();
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45777), point::Latitude(49.01855), point::Altitude(0)); // Nordtangente
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45768), point::Latitude(49.01852), point::Altitude(0)); // Nordtangente
  }
};

TEST_F(IntersectionTrafficLightElfNordtangente, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1, id2, id3;
  geoPoint = point::createGeoPoint(point::Longitude(8.457469344), point::Latitude(49.01830002), point::Altitude(5.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.457510429), point::Latitude(49.01830064), point::Altitude(5.));
  id2 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.457585752), point::Latitude(49.01830179), point::Altitude(3.));
  id3 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks(
    {expectedTrafficLight((uint64_t)id1), expectedTrafficLight((uint64_t)id2), expectedTrafficLight((uint64_t)id3)});
}

struct IntersectionTrafficLightElfToRusch : IntersectionTrafficLightSouthToWestTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45787), point::Latitude(49.02084), point::Altitude(0)); // An der Tagweide
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45782), point::Latitude(49.02073), point::Altitude(0)); // An der Tagweide
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45765), point::Latitude(49.02046), point::Altitude(0)); // Elfmorgenbruchstrasse
  }
  virtual point::GeoPoint getGeoToSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45758), point::Latitude(49.02055), point::Altitude(0)); // Elfmorgenbruchstrasse
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45746), point::Latitude(49.02073), point::Altitude(0)); // Ruschgraben
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45755), point::Latitude(49.02073), point::Altitude(0)); // Ruschgraben
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45807), point::Latitude(49.02059), point::Altitude(0)); // Am Storrenacker
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45795), point::Latitude(49.02057), point::Altitude(0)); // Am Storrenacker
  }
};

TEST_F(IntersectionTrafficLightElfToRusch, basic_checks)
{
  point::GeoPoint geoPoint;
  landmark::LandmarkId id1, id2;
  geoPoint = point::createGeoPoint(point::Longitude(8.457552561), point::Latitude(49.02053703), point::Altitude(3.));
  id1 = landmark::uniqueLandmarkId(geoPoint);
  geoPoint = point::createGeoPoint(point::Longitude(8.45766481), point::Latitude(49.02051672), point::Altitude(5.));
  id2 = landmark::uniqueLandmarkId(geoPoint);
  performBasicTrafficLightsChecks({expectedTrafficLight((uint64_t)id1), expectedTrafficLight((uint64_t)id2)});
}

} // namespace map
} // namespace ad
