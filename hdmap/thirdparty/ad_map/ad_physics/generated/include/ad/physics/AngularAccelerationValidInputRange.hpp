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
#include "ad/physics/AngularAcceleration.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given AngularAcceleration is within valid input range
 *
 * \param[in] input the AngularAcceleration as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if AngularAcceleration is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::physics::AngularAcceleration>::lowest() <= \c input <=
 * std::numeric_limits<::ad::physics::AngularAcceleration>::max()
 *       -1e2 <= \c input <= 1e2
 */
inline bool withinValidInputRange(::ad::physics::AngularAcceleration const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid()
    && (std::numeric_limits<::ad::physics::AngularAcceleration>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::physics::AngularAcceleration>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::AngularAcceleration)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::physics::AngularAcceleration>::lowest(),
                  std::numeric_limits<::ad::physics::AngularAcceleration>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange
      = (::ad::physics::AngularAcceleration(-1e2) <= input) && (input <= ::ad::physics::AngularAcceleration(1e2));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::physics::AngularAcceleration)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::physics::AngularAcceleration(-1e2),
                    ::ad::physics::AngularAcceleration(1e2)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
