// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include <ad/map/point/Operation.hpp>
#include "IntersectionYieldTests.hpp"
#include "MapSetup.hpp"

namespace ad {
namespace map {

struct IntersectionYieldAlbertHirtenwegTest : IntersectionYieldEastToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(point::Longitude(8.43931), point::Latitude(49.01834), point::Altitude(0)); // Hirtenweg
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return point::createGeoPoint(point::Longitude(8.43935), point::Latitude(49.01836), point::Altitude(0)); // Hirtenweg
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return point::createGeoPoint(point::Longitude(8.43960), point::Latitude(49.01831), point::Altitude(0)); // Hirtenweg
  }
  //  Bug going out 11724 should pass with the new map
  //  virtual point::GeoPoint getGeoToSouth() const override
  //  {
  //    return point::createGeoPoint(8.43958), point::Latitude(49.01826), point::Altitude(0));
  //  }

  virtual point::GeoPoint getGeoToSouth() const override
  {
    return point::createGeoPoint(point::Longitude(8.43951), point::Latitude(49.01824), point::Altitude(0)); // Hirtenweg
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(point::Longitude(8.43941), point::Latitude(49.01821), point::Altitude(0)); // Hirtenweg
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(point::Longitude(8.43938), point::Latitude(49.01823), point::Altitude(0)); // Hirtenweg
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.43949), point::Latitude(49.01838), point::Altitude(0)); // Albert-Nestler-Strasse
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.43953), point::Latitude(49.01837), point::Altitude(0)); // Albert-Nestler-Strasse
  }
};

TEST_F(IntersectionYieldAlbertHirtenwegTest, basic_checks)
{
  performBasicChecks();
}

struct IntersectionHasWayHaidAlbertTest : IntersectionYieldNorthToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4385623), point::Latitude(49.0169213), point::Altitude(0)); // Haid-und-Neu-Strasse
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
      point::Longitude(8.4381707), point::Latitude(49.0167026), point::Altitude(0)); // Haid-und-Neu-Strasse
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4382954), point::Latitude(49.0169924), point::Altitude(0)); // Albert-Nestler-Strasse
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4383419), point::Latitude(49.0169941), point::Altitude(0)); // Albert-Nestler-Strasse
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

TEST_F(IntersectionHasWayHaidAlbertTest, basic_checks)
{
  performBasicChecks();
}

struct IntersectionHasWayOstringHaidTest : IntersectionYieldNorthToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.43613), point::Latitude(49.01548), point::Altitude(0)); // Haid-und-Neu-Strasse
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
    return point::createGeoPoint(point::Longitude(8.43628), point::Latitude(49.01531), point::Altitude(0)); // Ostring
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(point::Longitude(8.43609), point::Latitude(49.01546), point::Altitude(0)); // From Aral
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

TEST_F(IntersectionHasWayOstringHaidTest, basic_checks)
{
  performBasicChecks();
}

struct IntersectionYieldOstringFuesslinTest : IntersectionYieldNorthToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4362616), point::Latitude(49.015290), point::Altitude(0)); // Ostring
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
      point::Longitude(8.4364418), point::Latitude(49.0150447), point::Altitude(0)); // Ostring
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4361606), point::Latitude(49.0150901), point::Altitude(0)); // Fuesslinstrasse
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4361450), point::Latitude(49.0151190), point::Altitude(0)); // Fuesslinstrasse
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

TEST_F(IntersectionYieldOstringFuesslinTest, basic_checks)
{
  performBasicChecks();
}

struct IntersectionYieldOstringDollmaetschTest : IntersectionYieldNorthToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4365557), point::Latitude(49.0148831), point::Altitude(0)); // Ostring
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
      point::Longitude(8.4367108), point::Latitude(49.0146541), point::Altitude(0)); // Ostring
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4363734), point::Latitude(49.0146839), point::Altitude(0)); // Dollmaetschstrasse
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4363598), point::Latitude(49.0147097), point::Altitude(0)); // Dollmaetschstrasse
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

TEST_F(IntersectionYieldOstringDollmaetschTest, basic_checks)
{
  performBasicChecks();
}

struct IntersectionYieldOstringSiegristTest : IntersectionYieldNorthToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4368291), point::Latitude(49.0144719), point::Altitude(0)); // Ostring
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
      point::Longitude(8.4369835), point::Latitude(49.0142286), point::Altitude(0)); // Ostring
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4367010), point::Latitude(49.0142909), point::Altitude(0)); // Siegriststrasse
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4366834), point::Latitude(49.0143180), point::Altitude(0)); // Siegriststrasse
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

TEST_F(IntersectionYieldOstringSiegristTest, basic_checks)
{
  performBasicChecks();
}

struct IntersectionYieldOstringOkenTest : IntersectionYieldNorthToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4371787), point::Latitude(49.0139298), point::Altitude(0)); // Ostring
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
      point::Longitude(8.4373020), point::Latitude(49.0136974), point::Altitude(0)); // Ostring
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4370445), point::Latitude(49.0137448), point::Altitude(0)); // Okenstrasse
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4370330), point::Latitude(49.0137719), point::Altitude(0)); // Okenstrasse
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

TEST_F(IntersectionYieldOstringOkenTest, basic_checks)
{
  performBasicChecks();
}

struct IntersectionYieldOstringZamenhofTest : IntersectionYieldNorthToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4374924), point::Latitude(49.0133539), point::Altitude(0)); // Ostring
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
      point::Longitude(8.4376130), point::Latitude(49.0131208), point::Altitude(0)); // Ostring
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4373311), point::Latitude(49.0131831), point::Altitude(0)); // Zamenhofstrasse
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4373135), point::Latitude(49.0132136), point::Altitude(0)); // Zamenhofstrasse
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

TEST_F(IntersectionYieldOstringZamenhofTest, basic_checks)
{
  performBasicChecks();
}

struct IntersectionYieldOstringStriederTest : IntersectionYieldNorthToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4390867), point::Latitude(49.0109790), point::Altitude(0)); // Ostring
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
      point::Longitude(8.4393944), point::Latitude(49.0107432), point::Altitude(0)); // Ostring
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4390027), point::Latitude(49.0107601), point::Altitude(0)); // Striederstrasse
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.4389932), point::Latitude(49.0107872), point::Altitude(0)); // Striederstrasse
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

TEST_F(IntersectionYieldOstringStriederTest, basic_checks)
{
  performBasicChecks();
}

struct IntersectionYieldOstringWeinwegTest : IntersectionYieldNorthToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(point::Longitude(8.442135), point::Latitude(49.008290), point::Altitude(0)); // Ostring
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return point::createGeoPoint(point::Longitude(8.442281), point::Latitude(49.008254), point::Altitude(0)); // Ostring
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return point::createGeoPoint(point::Longitude(8.442108), point::Latitude(49.007866), point::Altitude(0)); // Ostring
  }

  virtual point::GeoPoint getGeoToSouth() const override
  {
    return point::createGeoPoint(point::Longitude(8.441964), point::Latitude(49.007903), point::Altitude(0)); // Ostring
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
    return point::createGeoPoint(point::Longitude(8.442547), point::Latitude(49.008046), point::Altitude(0)); // Weinweg
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return point::createGeoPoint(point::Longitude(8.442547), point::Latitude(49.007983), point::Altitude(0)); // Weinweg
  }
};

TEST_F(IntersectionYieldOstringWeinwegTest, basic_checks)
{
  performBasicChecks();
}

struct IntersectionHasWayElfmorgenDurlachCentreEntryTest : IntersectionYieldNorthToSouthTest
{
  virtual void prepareMap() const override
  {
    ::map_setup::prepareMapTpkPfzDrive();
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.44858), point::Latitude(49.00611), point::Altitude(0)); // Durlach Centre Entry
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
      point::Longitude(8.44878), point::Latitude(49.00627), point::Altitude(0)); // Elfmorgenbruchstrasse
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

TEST_F(IntersectionHasWayElfmorgenDurlachCentreEntryTest, basic_checks)
{
  performBasicChecks();
}

} // namespace map
} // namespace ad
