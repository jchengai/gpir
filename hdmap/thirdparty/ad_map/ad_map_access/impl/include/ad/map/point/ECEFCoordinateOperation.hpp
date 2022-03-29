// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/point/ECEFCoordinate.hpp"
#include "ad/physics/Distance.hpp"

/**
 * @brief multiplies a ECEFCoordinate with a scalar
 * @param[in] a coordinate a
 * @param[in] b scalar b
 */
inline ::ad::map::point::ECEFCoordinate operator*(
    ::ad::map::point::ECEFCoordinate const &a,
    ::ad::physics::Distance const &b) {
  return ::ad::map::point::ECEFCoordinate(static_cast<double>(a) *
                                          static_cast<double>(b));
}

/**
 * @brief multiplies a ECEFCoordinate with a scalar
 * @param[in] b scalar b
 * @param[in] a coordinate a
 */
inline ::ad::map::point::ECEFCoordinate operator*(
    ::ad::physics::Distance const &b,
    ::ad::map::point::ECEFCoordinate const &a) {
  return ::ad::map::point::ECEFCoordinate(static_cast<double>(a) *
                                          static_cast<double>(b));
}

/**
 * @brief division of a ECEFCoordinate by a scalar
 * @param[in] a coordinate a
 * @param[in] b scalar b
 */
inline ::ad::map::point::ECEFCoordinate operator/(
    ::ad::map::point::ECEFCoordinate const &a,
    ::ad::physics::Distance const &b) {
  return a / static_cast<double>(b);
}

/**
 * @brief multiplies ECEFCoordinates
 * @param[in] a coordinate a
 * @param[in] b coordinate b
 */
inline ::ad::map::point::ECEFCoordinate operator*(
    ::ad::map::point::ECEFCoordinate const &a,
    ::ad::map::point::ECEFCoordinate const &b) {
  return ::ad::map::point::ECEFCoordinate(static_cast<double>(a) *
                                          static_cast<double>(b));
}
