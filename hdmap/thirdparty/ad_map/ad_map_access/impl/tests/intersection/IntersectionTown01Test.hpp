// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "SyntheticIntersectionTestBase.hpp"

namespace ad {
namespace map {

struct IntersectionTown01WestToSouthTest : virtual SyntheticIntersectionTestBase
{
  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return point::createGeoPoint(point::Longitude(0.0032321), point::Latitude(0.0000165612), point::Altitude(0));
  }

  virtual point::GeoPoint getGeoToSouth() const override
  {
    return point::createGeoPoint(point::Longitude(0.0032321), point::Latitude(-0.0000169493), point::Altitude(0));
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(point::Longitude(0.0030455), point::Latitude(-0.0002023962), point::Altitude(0));
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(point::Longitude(0.003000), point::Latitude(-0.0002566106), point::Altitude(0));
  }

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(point::Longitude(0.0028), point::Latitude(-0.0000170657), point::Altitude(0));
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return point::createGeoPoint(point::Longitude(0.0028), point::Latitude(0.0000237813), point::Altitude(0));
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

} // namespace map
} // namespace ad
