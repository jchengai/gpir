// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/point/Operation.hpp>
#include "IntersectionPriorityToRightTests.hpp"
#include "MapSetup.hpp"

namespace ad {
namespace map {

struct IntersectionPrioRightEmmyAlbertTest : IntersectionPriorityToRightWestToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44070), point::Latitude(49.01990), point::Altitude(0)); // Albert-Nestler-Strasse
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44073), point::Latitude(49.01988), point::Altitude(0)); // Albert-Nestler-Strasse
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44063), point::Latitude(49.01976), point::Altitude(0)); // Albert-Nestler-Strasse
  }
  virtual point::GeoPoint getGeoToSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44059), point::Latitude(49.01978), point::Altitude(0)); // Albert-Nestler-Strasse
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44056), point::Latitude(49.01986), point::Altitude(0)); // Emmy-Noether-Strasse
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44058), point::Latitude(49.01988), point::Altitude(0)); // Emmy-Noether-Strasse
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44076), point::Latitude(49.01981), point::Altitude(0)); // Emmy-Noether-Strasse
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44074), point::Latitude(49.01978), point::Altitude(0)); // Emmy-Noether-Strasse
  }
};

TEST_F(IntersectionPrioRightEmmyAlbertTest, basic_checks)
{
  performBasicChecks();
}

struct IntersectionPrioRightAlbertAdaTest : IntersectionPriorityToRightNorthToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44017), point::Latitude(49.01927), point::Altitude(0)); // Albert-Nestler-Strasse
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44021), point::Latitude(49.01926), point::Altitude(0)); // Albert-Nestler-Strasse
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44010), point::Latitude(49.01912), point::Altitude(0)); // Albert-Nestler-Strasse
  }
  virtual point::GeoPoint getGeoToSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44006), point::Latitude(49.01913), point::Altitude(0)); // Albert-Nestler-Strasse
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44005), point::Latitude(49.01923), point::Altitude(0)); // Ada-Lovelace-Strasse
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44008), point::Latitude(49.01926), point::Altitude(0)); // Ada-Lovelace-Strasse
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

TEST_F(IntersectionPrioRightAlbertAdaTest, basic_checks)
{
  performBasicChecks();
}

struct IntersectionPrioRightRuschPfizerTest : IntersectionPriorityToRightSouthToEastTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45331), point::Latitude(49.02390), point::Altitude(0)); // Ruschgraben
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45336), point::Latitude(49.02390), point::Altitude(0)); // Ruschgraben
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45332), point::Latitude(49.02371), point::Altitude(0)); // Ruschgraben
  }
  virtual point::GeoPoint getGeoToSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45328), point::Latitude(49.02375), point::Altitude(0)); // Ruschgraben
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
      point::Longitude(8.45343), point::Latitude(49.02384), point::Altitude(0)); // Pfizerstrasse
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45342), point::Latitude(49.02381), point::Altitude(0)); // Pfizerstrasse
  }
};

TEST_F(IntersectionPrioRightRuschPfizerTest, basic_checks)
{
  performBasicChecks();
}

struct IntersectionPrioRightPfizerAnDerTest : IntersectionPriorityToRightWestToEastTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45434), point::Latitude(49.02370), point::Altitude(0)); // An der alten Bach,
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45437), point::Latitude(49.02371), point::Altitude(0)); // An der alten Bach,
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return point::GeoPoint();
  }
  virtual point::GeoPoint getGeoToSouth() const override
  {
    return point::GeoPoint();
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45416), point::Latitude(49.02361), point::Altitude(0)); // Pfizerstrasse
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45413), point::Latitude(49.02365), point::Altitude(0)); // Pfizerstrasse
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45443), point::Latitude(49.02355), point::Altitude(0)); // Pfizerstrasse
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45441), point::Latitude(49.02353), point::Altitude(0)); // Pfizerstrasse
  }
};

TEST_F(IntersectionPrioRightPfizerAnDerTest, basic_checks)
{
  performBasicChecks();
}

} // namespace map
} // namespace ad
