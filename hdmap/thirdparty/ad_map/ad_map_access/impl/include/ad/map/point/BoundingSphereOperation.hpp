// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/point/BoundingSphere.hpp"
#include "ad/map/point/ECEFOperation.hpp"
#include "ad/map/point/Geometry.hpp"

namespace ad {
namespace map {
namespace point {

/**
 * @brief Computes distance between BoundingSpheres.
 * @returns Distance between two bounding spheres in meters.
 */
inline physics::Distance distance(BoundingSphere const &left,
                                  BoundingSphere const &right) {
  physics::Distance const distanceCenter = distance(left.center, right.center);
  return std::max(physics::Distance(0u),
                  distanceCenter - left.radius - right.radius);
}

/**
 * @brief calculate the bounding sphere of two edges
 */
BoundingSphere calcBoundingSphere(Geometry const &edgeLeft,
                                  Geometry const &edgeRight);

}  // namespace point
}  // namespace map
}  // namespace ad

/**
 * @brief calculate the union of two BoundingSpheres
 *
 * @param[in] a BoundingSphere a
 * @param[in] b BoundingSphere b
 *
 * @returns union BoundingSpere containing both: a U b
 */
::ad::map::point::BoundingSphere operator+(
    ::ad::map::point::BoundingSphere const &a,
    ::ad::map::point::BoundingSphere const &b);
