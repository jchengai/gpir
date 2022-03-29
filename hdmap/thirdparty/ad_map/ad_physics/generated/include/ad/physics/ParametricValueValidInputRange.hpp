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
#include "ad/physics/ParametricValue.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given ParametricValue is within valid input range
 *
 * \param[in] input the ParametricValue as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if ParametricValue is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::physics::ParametricValue>::lowest() <= \c input <=
 * std::numeric_limits<::ad::physics::ParametricValue>::max()
 *       0. <= \c input <= 1.
 */
inline bool withinValidInputRange(::ad::physics::ParametricValue const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::physics::ParametricValue>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::physics::ParametricValue>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::ParametricValue)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::physics::ParametricValue>::lowest(),
                  std::numeric_limits<::ad::physics::ParametricValue>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange = (::ad::physics::ParametricValue(0.) <= input) && (input <= ::ad::physics::ParametricValue(1.));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::physics::ParametricValue)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::physics::ParametricValue(0.),
                    ::ad::physics::ParametricValue(1.)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
