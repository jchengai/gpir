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

#pragma once

#include <cmath>
#include <limits>
#include "ad/physics/DistanceSquared.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given DistanceSquared is within valid input range
 *
 * \param[in] input the DistanceSquared as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if DistanceSquared is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::physics::DistanceSquared>::lowest() <= \c input <=
 * std::numeric_limits<::ad::physics::DistanceSquared>::max()
 *       0. <= \c input <= 1e12
 */
inline bool withinValidInputRange(::ad::physics::DistanceSquared const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::physics::DistanceSquared>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::physics::DistanceSquared>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::DistanceSquared)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::physics::DistanceSquared>::lowest(),
                  std::numeric_limits<::ad::physics::DistanceSquared>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange
      = (::ad::physics::DistanceSquared(0.) <= input) && (input <= ::ad::physics::DistanceSquared(1e12));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::physics::DistanceSquared)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::physics::DistanceSquared(0.),
                    ::ad::physics::DistanceSquared(1e12)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
