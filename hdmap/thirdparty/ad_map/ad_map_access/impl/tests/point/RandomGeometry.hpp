// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <ad/map/point/Operation.hpp>
#include <cstdlib>

using namespace ad;
using namespace ad::map;

inline point::ECEFPoint randECEFPoint()
{
  double rndLat = std::rand() / static_cast<double>(RAND_MAX);
  double rndLon = std::rand() / static_cast<double>(RAND_MAX);
  double rndAlt = std::rand() / static_cast<double>(RAND_MAX);

  auto lat = point::Latitude(-90.0 + 180.0 * rndLat);
  auto lon = point::Longitude(-180.0 + 360.0 * rndLon);
  auto alt = point::Altitude(-100.0 + 3100.0 * rndAlt);
  return point::toECEF(point::createGeoPoint(lon, lat, alt));
}

inline point::Geometry randGeometry(const point::ECEFPoint &pt0, size_t points, uint32_t seed)
{
  point::ECEFEdge pts;
  std::srand(seed);
  point::ECEFPoint pt = pt0;
  pts.push_back(pt);
  for (size_t i = 1; i < points; i++)
  {
    auto offsetPt = point::createECEFPoint(std::rand() % 10, std::rand() % 10, std::rand() % 10);
    pt = pt + offsetPt;
    pts.push_back(pt);
  }
  return point::createGeometry(pts, false);
}

inline point::Geometry randGeometry(size_t points, uint32_t seed)
{
  std::srand(seed ^ 0x12345678);
  return randGeometry(randECEFPoint(), points, seed);
}
