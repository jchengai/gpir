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
#include "ad/physics/SpeedSquared.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given SpeedSquared is within valid input range
 *
 * \param[in] input the SpeedSquared as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if SpeedSquared is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::physics::SpeedSquared>::lowest() <= \c input <=
 * std::numeric_limits<::ad::physics::SpeedSquared>::max()
 *       -1e4 <= \c input <= 1e4
 */
inline bool withinValidInputRange(::ad::physics::SpeedSquared const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::physics::SpeedSquared>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::physics::SpeedSquared>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::SpeedSquared)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::physics::SpeedSquared>::lowest(),
                  std::numeric_limits<::ad::physics::SpeedSquared>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange = (::ad::physics::SpeedSquared(-1e4) <= input) && (input <= ::ad::physics::SpeedSquared(1e4));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::physics::SpeedSquared)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::physics::SpeedSquared(-1e4),
                    ::ad::physics::SpeedSquared(1e4)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
