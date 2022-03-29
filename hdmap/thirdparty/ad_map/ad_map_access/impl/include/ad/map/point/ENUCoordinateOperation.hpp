// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/point/ENUCoordinate.hpp"
#include "ad/physics/Distance.hpp"

/**
 * @brief multiplies a ENUCoordinate with a scalar
 * @param[in] a coordinate a
 * @param[in] b scalar b
 */
inline ::ad::map::point::ENUCoordinate operator*(::ad::map::point::ENUCoordinate const &a,
                                                 ::ad::physics::Distance const &b)
{
  return ::ad::map::point::ENUCoordinate(static_cast<double>(a) * static_cast<double>(b));
}

/**
 * @brief multiplies a ENUCoordinate with a scalar
 * @param[in] b scalar b
 * @param[in] a coordinate a
 */
inline ::ad::map::point::ENUCoordinate operator*(::ad::physics::Distance const &b,
                                                 ::ad::map::point::ENUCoordinate const &a)
{
  return ::ad::map::point::ENUCoordinate(static_cast<double>(a) * static_cast<double>(b));
}

/**
 * @brief division of a ECEFCoordinate by a scalar
 * @param[in] a coordinate a
 * @param[in] b scalar b
 */
inline ::ad::map::point::ENUCoordinate operator/(::ad::map::point::ENUCoordinate const &a,
                                                 ::ad::physics::Distance const &b)
{
  return a / static_cast<double>(b);
}

/**
 * @brief multiplies ENUCoordinate
 * @param[in] a coordinate a
 * @param[in] b coordinate b
 */
inline ::ad::map::point::ENUCoordinate operator*(::ad::map::point::ENUCoordinate const &a,
                                                 ::ad::map::point::ENUCoordinate const &b)
{
  return ::ad::map::point::ENUCoordinate(static_cast<double>(a) * static_cast<double>(b));
}
