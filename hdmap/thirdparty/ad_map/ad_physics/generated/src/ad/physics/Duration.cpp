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

#include "ad/physics/Duration.hpp"
#include "ad/physics/DurationSquared.hpp"

/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace physics
 */
namespace physics {

const double Duration::cMinValue = -1e6;

const double Duration::cMaxValue = 1e6;

const double Duration::cPrecisionValue = 1e-3;

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

::ad::physics::DurationSquared Duration::operator*(const Duration &other) const
{
  ensureValid();
  other.ensureValid();
  ::ad::physics::DurationSquared const result(mDuration * other.mDuration);
  result.ensureValid(); // LCOV_EXCL_BR_LINE On correct definition of squarerooted type, this cannot not happen
  return result;
}

} // namespace physics
} // namespace ad
