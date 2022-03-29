// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/point/ENUEdgeValidInputRange.hpp"
#include "ad/map/point/ENUPointValidInputRange.hpp"
#include "ad/map/point/PointOperation.hpp"

namespace ad {
namespace map {
namespace point {

/**
 * @brief checks if the given ENUPoint is valid
 *
 * The point is valid if it's within valid input range.
 */
inline bool isValid(ENUPoint const &point, bool const logErrors = true) {
  return withinValidInputRange(point, logErrors);
}

/**
 * @brief checks if the given ENUEdge is valid
 *
 * The edge is valid if it's within valid input range.
 */
inline bool isValid(ENUEdge const &edge, bool const logErrors = true) {
  return withinValidInputRange(edge, logErrors);
}

/**
 * @brief create a ENUPoint
 *
 * @param[in] x x-coodinate of the point
 * @param[in] y y-coodinate of the point
 * @param[in] z z-coodinate of the point
 */
inline ENUPoint createENUPoint(double const x, double const y, double const z) {
  ENUPoint result;
  result.x = ENUCoordinate(x);
  result.y = ENUCoordinate(y);
  result.z = ENUCoordinate(z);
  return result;
}

/**
 * @brief create a ENUPoint
 *
 * @param[in] x x-coodinate of the point
 * @param[in] y y-coodinate of the point
 * @param[in] z z-coodinate of the point
 */
inline ENUPoint createENUPoint(ENUCoordinate const x, ENUCoordinate const y,
                               ENUCoordinate const z) {
  ENUPoint result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

/**
 * @brief get ENUPoint defining the East-Axis
 */
inline ENUPoint getEnuEastAxis() {
  ENUPoint result;
  result.x = ENUCoordinate(1.);
  result.y = ENUCoordinate(0.);
  result.z = ENUCoordinate(0.);
  return result;
}

/**
 * @brief get ENUPoint defining the North-Axis
 */
inline ENUPoint getEnuNorthAxis() {
  ENUPoint result;
  result.x = ENUCoordinate(0.);
  result.y = ENUCoordinate(1.);
  result.z = ENUCoordinate(0.);
  return result;
}

/**
 * @brief get ENUPoint defining the Up-Axis
 */
inline ENUPoint getEnuUpAxis() {
  ENUPoint result;
  result.x = ENUCoordinate(0.);
  result.y = ENUCoordinate(0.);
  result.z = ENUCoordinate(1.);
  return result;
}

/**
 * @brief Computes distance between ENU points.
 * @returns Distance between two points in meters.
 */
inline physics::Distance distance(ENUPoint const &point,
                                  ENUPoint const &other) {
  return vectorLength(vectorSub(point, other));
}

/** @brief calculate the length of the provided border as distance value
 *
 * Length calculation is performed within Cartesian ECEF coordinate frame.
 */
physics::Distance calcLength(ENUEdge const &edge);

}  // namespace point
}  // namespace map
}  // namespace ad

/**
 * @brief calculate the dot product of two ENUPoint vectors
 *
 * @param[in] a vector a
 * @param[in] b vector b
 *
 * @returns value d = a * b
 */
inline double operator*(::ad::map::point::ENUPoint const &a,
                        ::ad::map::point::ENUPoint const &b) {
  return ::ad::map::point::vectorDotProduct(a, b);
}

/**
 * @brief multiplies a ENUPoint vector with a scalar
 * @param[in] a vector a
 * @param[in] b scalar b
 */
inline ::ad::map::point::ENUPoint operator*(::ad::map::point::ENUPoint const &a,
                                            ::ad::physics::Distance const &b) {
  return ::ad::map::point::vectorMultiplyScalar(a, b);
}

/**
 * @brief multiplies a ENUPoint vector with a scalar
 * @param[in] b scalar b
 * @param[in] a vector a
 */
inline ::ad::map::point::ENUPoint operator*(
    ::ad::physics::Distance const &b, ::ad::map::point::ENUPoint const &a) {
  return ::ad::map::point::vectorMultiplyScalar(a, b);
}

/**
 * @brief multiplies a ENUPoint vector with a scalar
 * @param[in] a vector a
 * @param[in] b scalar b
 */
inline ::ad::map::point::ENUPoint operator*(::ad::map::point::ENUPoint const &a,
                                            double const &b) {
  return ::ad::map::point::vectorMultiplyScalar(a, b);
}

/**
 * @brief multiplies a ENUPoint vector with a scalar
 * @param[in] b scalar b
 * @param[in] a vector a
 */
inline ::ad::map::point::ENUPoint operator*(
    double const &b, ::ad::map::point::ENUPoint const &a) {
  return ::ad::map::point::vectorMultiplyScalar(a, b);
}

/**
 * @brief add two ENUPoint vectors
 *
 * @param[in] a vector a
 * @param[in] b vector b
 *
 * @returns vector c = a + b
 */
inline ::ad::map::point::ENUPoint operator+(
    ::ad::map::point::ENUPoint const &a, ::ad::map::point::ENUPoint const &b) {
  return ::ad::map::point::vectorAdd(a, b);
}

/**
 * @brief subtract two ENUPoint vectors from each right
 *
 * @param[in] a vector a
 * @param[in] b vector b
 *
 * @returns c = a - b
 */
inline ::ad::map::point::ENUPoint operator-(
    ::ad::map::point::ENUPoint const &a, ::ad::map::point::ENUPoint const &b) {
  return ::ad::map::point::vectorSub(a, b);
}
