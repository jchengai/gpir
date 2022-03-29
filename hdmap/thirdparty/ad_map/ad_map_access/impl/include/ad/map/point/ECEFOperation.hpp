// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/point/ECEFEdgeValidInputRange.hpp"
#include "ad/map/point/ECEFPointValidInputRange.hpp"
#include "ad/map/point/PointOperation.hpp"

namespace ad {
namespace map {
namespace point {

/**
 * @brief checks if the given ECEFPoint is valid
 *
 * The point is valid if it's within valid input range.
 */
inline bool isValid(ECEFPoint const &point, bool const logErrors = true) {
  return withinValidInputRange(point, logErrors);
}

/**
 * @brief checks if the given ECEFEdge is valid
 *
 * The point is valid if it's within valid input range.
 */
inline bool isValid(ECEFEdge const &edge, bool const logErrors = true) {
  return withinValidInputRange(edge, logErrors);
}

/**
 * @brief create a ECEFPoint
 *
 * @param[in] x x-coodinate of the point
 * @param[in] y y-coodinate of the point
 * @param[in] z z-coodinate of the point
 */
inline ECEFPoint createECEFPoint(double const x, double const y,
                                 double const z) {
  ECEFPoint result;
  result.x = ECEFCoordinate(x);
  result.y = ECEFCoordinate(y);
  result.z = ECEFCoordinate(z);
  return result;
}

/**
 * @brief create a ECEFPoint
 *
 * @param[in] x x-coodinate of the point
 * @param[in] y y-coodinate of the point
 * @param[in] z z-coodinate of the point
 */
inline ECEFPoint createECEFPoint(ECEFCoordinate const x, ECEFCoordinate const y,
                                 ECEFCoordinate const z) {
  ECEFPoint result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

/**
 * @brief Computes distance between ECEF points.
 * @returns Distance between two points in meters.
 */
inline physics::Distance distance(ECEFPoint const &point,
                                  ECEFPoint const &other) {
  return vectorLength(vectorSub(point, other));
}

/** @brief calculate the length of the provided border as distance value
 *
 * Length calculation is performed within Cartesian ECEF coordinate frame.
 */
physics::Distance calcLength(ECEFEdge const &edge);

}  // namespace point
}  // namespace map
}  // namespace ad

/**
 * @brief calculate the dot product of two ECEFPoint vectors
 *
 * @param[in] a vector a
 * @param[in] b vector b
 *
 * @returns value d = a * b
 */
inline double operator*(::ad::map::point::ECEFPoint const &a,
                        ::ad::map::point::ECEFPoint const &b) {
  return ::ad::map::point::vectorDotProduct(a, b);
}

/**
 * @brief multiplies a ECEFPoint vector with a scalar
 * @param[in] a vector a
 * @param[in] b scalar b
 */
inline ::ad::map::point::ECEFPoint operator*(
    ::ad::map::point::ECEFPoint const &a, ::ad::physics::Distance const &b) {
  return ::ad::map::point::vectorMultiplyScalar(a, b);
}

/**
 * @brief multiplies a ECEFPoint vector with a scalar
 * @param[in] b scalar b
 * @param[in] a vector a
 */
inline ::ad::map::point::ECEFPoint operator*(
    ::ad::physics::Distance const &b, ::ad::map::point::ECEFPoint const &a) {
  return ::ad::map::point::vectorMultiplyScalar(a, b);
}

/**
 * @brief multiplies a ECEFPoint vector with a scalar
 * @param[in] a vector a
 * @param[in] b scalar b
 */
inline ::ad::map::point::ECEFPoint operator*(
    ::ad::map::point::ECEFPoint const &a, double const &b) {
  return ::ad::map::point::vectorMultiplyScalar(a, b);
}

/**
 * @brief multiplies a ECEFPoint vector with a scalar
 * @param[in] b scalar b
 * @param[in] a vector a
 */
inline ::ad::map::point::ECEFPoint operator*(
    double const &b, ::ad::map::point::ECEFPoint const &a) {
  return ::ad::map::point::vectorMultiplyScalar(a, b);
}

/**
 * @brief add two ECEFPoint vectors
 *
 * @param[in] a vector a
 * @param[in] b vector b
 *
 * @returns vector c = a + b
 */
inline ::ad::map::point::ECEFPoint operator+(
    ::ad::map::point::ECEFPoint const &a,
    ::ad::map::point::ECEFPoint const &b) {
  return ::ad::map::point::vectorAdd(a, b);
}

/**
 * @brief subtract two ECEFPoint vectors from each right
 *
 * @param[in] a vector a
 * @param[in] b vector b
 *
 * @returns c = a - b
 */
inline ::ad::map::point::ECEFPoint operator-(
    ::ad::map::point::ECEFPoint const &a,
    ::ad::map::point::ECEFPoint const &b) {
  return ::ad::map::point::vectorSub(a, b);
}
