// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/test_support/ArtificialIntersectionTestBase.hpp"

#include "ad/map/access/Operation.hpp"
#include "ad/map/point/Operation.hpp"

namespace ad {
namespace map {
namespace test_support {

ArtificialIntersectionTestBase::ArtificialIntersectionTestBase()
  : IntersectionTestBase()
  , mPoint_01h(point::createGeoPoint(point::Longitude(8.43748), point::Latitude(49.01890), point::Altitude(0)))
  , mPoint_02h(point::createGeoPoint(point::Longitude(8.43770), point::Latitude(49.01870), point::Altitude(0)))
  , mPoint_04h(point::createGeoPoint(point::Longitude(8.43770), point::Latitude(49.01867), point::Altitude(0)))
  , mPoint_05h(point::createGeoPoint(point::Longitude(8.43747), point::Latitude(49.01850), point::Altitude(0)))
  , mPoint_07h(point::createGeoPoint(point::Longitude(8.43743), point::Latitude(49.01850), point::Altitude(0)))
  , mPoint_08h(point::createGeoPoint(point::Longitude(8.43720), point::Latitude(49.01867), point::Altitude(0)))
  , mPoint_10h(point::createGeoPoint(point::Longitude(8.43720), point::Latitude(49.01869), point::Altitude(0)))
  , mPoint_11h(point::createGeoPoint(point::Longitude(8.43743), point::Latitude(49.01890), point::Altitude(0)))
{
}

point::GeoPoint ArtificialIntersectionTestBase::getGeoFromNorth() const
{
  if (access::isRightHandedTraffic())
  {
    return mPoint_11h;
  }
  else
  {
    return mPoint_01h;
  }
}

point::GeoPoint ArtificialIntersectionTestBase::getGeoToNorth() const
{
  if (access::isRightHandedTraffic())
  {
    return mPoint_01h;
  }
  else
  {
    return mPoint_11h;
  }
}

point::GeoPoint ArtificialIntersectionTestBase::getGeoFromSouth() const
{
  if (access::isRightHandedTraffic())
  {
    return mPoint_05h;
  }
  else
  {
    return mPoint_07h;
  }
}

point::GeoPoint ArtificialIntersectionTestBase::getGeoToSouth() const
{
  if (access::isRightHandedTraffic())
  {
    return mPoint_07h;
  }
  else
  {
    return mPoint_05h;
  }
}

point::GeoPoint ArtificialIntersectionTestBase::getGeoFromWest() const
{
  if (access::isRightHandedTraffic())
  {
    return mPoint_08h;
  }
  else
  {
    return mPoint_10h;
  }
}

point::GeoPoint ArtificialIntersectionTestBase::getGeoToWest() const
{
  if (access::isRightHandedTraffic())
  {
    return mPoint_10h;
  }
  else
  {
    return mPoint_08h;
  }
}

point::GeoPoint ArtificialIntersectionTestBase::getGeoFromEast() const
{
  if (access::isRightHandedTraffic())
  {
    return mPoint_02h;
  }
  else
  {
    return mPoint_04h;
  }
}

point::GeoPoint ArtificialIntersectionTestBase::getGeoToEast() const
{
  if (access::isRightHandedTraffic())
  {
    return mPoint_04h;
  }
  else
  {
    return mPoint_02h;
  }
}

} // namespace test_support
} // namespace map
} // namespace ad
