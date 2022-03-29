/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (C) 2018-2020 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

/**
 * Generated file
 * @file
 *
 * Generator Version : 11.0.0-1997
 */

#include "ad/physics/Distance.hpp"
#include "ad/physics/DistanceSquared.hpp"

/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace physics
 */
namespace physics {

const double Distance::cMinValue = -1e9;

const double Distance::cMaxValue = 1e9;

const double Distance::cPrecisionValue = 1e-3;

} // namespace physics
} // namespace ad
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace physics
 */
namespace physics {

::ad::physics::DistanceSquared Distance::operator*(const Distance &other) const
{
  ensureValid();
  other.ensureValid();
  ::ad::physics::DistanceSquared const result(mDistance * other.mDistance);
  result.ensureValid(); // LCOV_EXCL_BR_LINE On correct definition of squarerooted type, this cannot not happen
  return result;
}

} // namespace physics
} // namespace ad
