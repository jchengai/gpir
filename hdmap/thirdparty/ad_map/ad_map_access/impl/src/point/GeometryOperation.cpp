// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/point/GeometryOperation.hpp"

#include <algorithm>

#include "ad/map/access/Logging.hpp"
#include "ad/map/access/Operation.hpp"
// #include "ad/map/point/EdgeOperation.hpp"
#include "ad/map/point/Operation.hpp"

namespace ad {
namespace map {
namespace point {

Geometry createGeometry(const ECEFEdge &points, bool closed) {
  Geometry geometry;
  geometry.isClosed = closed;
  geometry.ecefEdge = points;
  geometry.private_enuEdgeCache.enuVersion = 0;
  geometry.isValid = (points.size() >= 2) && isValid(points);
  geometry.length = calcLength(points);
  return geometry;
}

ENUEdge getCachedENUEdge(Geometry const &geometry) {
  auto coordinateTransform = access::getCoordinateTransform();
  ENUEdgeCache &mutable_enuEdgeCache =
      const_cast<Geometry *>(&geometry)->private_enuEdgeCache;
  if (!coordinateTransform || geometry.private_enuEdgeCache.enuVersion !=
                                  coordinateTransform->getENURef()) {
    mutable_enuEdgeCache.enuEdge.clear();
  }
  if (geometry.private_enuEdgeCache.enuEdge.empty()) {
    if (coordinateTransform) {
      if (coordinateTransform->isENUValid()) {
        mutable_enuEdgeCache.enuVersion = coordinateTransform->getENURef();
        coordinateTransform->convert(geometry.ecefEdge,
                                     mutable_enuEdgeCache.enuEdge);
      } else {
        access::getLogger()->error(
            "Geometry::GetENU: ENU Reference Point not defined.");
      }
    } else {
      access::getLogger()->error(
          "Geometry::GetENU: Coordinate transformations not defined.");
    }
  }
  return geometry.private_enuEdgeCache.enuEdge;
}

bool calculateCachedENUCenterLine(Geometry const &left, Geometry const &right,
                                  ENUEdge &center) {
  if (left.private_enuEdgeCache.enuEdge.empty() ||
      right.private_enuEdgeCache.enuEdge.empty()) {
    access::getLogger()->error("left or right is empty");
    return false;
  }

  center = ad::map::point::getLateralAlignmentEdge(
      left.private_enuEdgeCache.enuEdge, left.length,
      right.private_enuEdgeCache.enuEdge, right.length,
      ad::physics::ParametricValue(0.5));
  return true;
}

/////////////
// Operations

bool isSuccessor(Geometry const &geometry, const Geometry &other) {
  if (geometry.ecefEdge.empty() || other.ecefEdge.empty()) {
    return false;
  } else if (geometry.ecefEdge.front() == other.ecefEdge.back()) {
    return true;
  } else if (geometry.ecefEdge.back() == other.ecefEdge.back()) {
    return true;
  } else {
    return false;
  }
}

bool isPredecessor(Geometry const &geometry, const Geometry &other) {
  if (geometry.ecefEdge.empty() || other.ecefEdge.empty()) {
    return false;
  } else if (geometry.ecefEdge.front() == other.ecefEdge.front()) {
    return true;
  } else if (geometry.ecefEdge.back() == other.ecefEdge.front()) {
    return true;
  } else {
    return false;
  }
}

bool haveSameStart(Geometry const &geometry, const Geometry &other) {
  if (geometry.ecefEdge.empty() || other.ecefEdge.empty()) {
    return false;
  } else {
    return geometry.ecefEdge.front() == other.ecefEdge.front();
  }
}

bool haveSameEnd(Geometry const &geometry, const Geometry &other) {
  if (geometry.ecefEdge.empty() || other.ecefEdge.empty()) {
    return false;
  } else {
    return geometry.ecefEdge.back() == other.ecefEdge.back();
  }
}

point::ECEFPoint getParametricPoint(Geometry const &geometry,
                                    const physics::ParametricValue &t) {
  return point::getParametricPoint(geometry.ecefEdge, geometry.length, t);
}

void getParametricRange(Geometry const &geometry,
                        const physics::ParametricRange &trange,
                        ECEFEdge &outputEdge, const bool revertOrder) {
  outputEdge =
      point::getParametricRange(geometry.ecefEdge, geometry.length, trange);
  if (revertOrder) {
    std::reverse(outputEdge.begin(), outputEdge.end());
  }
}

void getParametricRange(Geometry const &geometry,
                        const physics::ParametricRange &trange,
                        GeoEdge &outputEdge, const bool revertOrder) {
  ECEFEdge const ecefEdge =
      point::getParametricRange(geometry.ecefEdge, geometry.length, trange);
  outputEdge = toGeo(ecefEdge);
  if (revertOrder) {
    std::reverse(outputEdge.begin(), outputEdge.end());
  }
}

void getParametricRange(Geometry const &geometry,
                        const physics::ParametricRange &trange,
                        ENUEdge &outputEdge, const bool revertOrder) {
  outputEdge = point::getParametricRange(getCachedENUEdge(geometry),
                                         geometry.length, trange);
  if (revertOrder) {
    std::reverse(outputEdge.begin(), outputEdge.end());
  }
}

ECEFEdge getMiddleEdge(Geometry const &geometry, Geometry const &other) {
  return point::getLateralAlignmentEdge(geometry.ecefEdge, geometry.length,
                                        other.ecefEdge, other.length,
                                        physics::ParametricValue(0.5));
}

physics::ParametricValue findNearestPointOnEdge(Geometry const &geometry,
                                                const point::ECEFPoint &pt) {
  return findNearestPointOnEdge(geometry.ecefEdge, geometry.length, pt);
}

}  // namespace point
}  // namespace map
}  // namespace ad
