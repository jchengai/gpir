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
#include "ad/physics/Speed.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given Speed is within valid input range
 *
 * \param[in] input the Speed as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if Speed is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::physics::Speed>::lowest() <= \c input <=
 * std::numeric_limits<::ad::physics::Speed>::max()
 *       -100. <= \c input <= 100.
 */
inline bool withinValidInputRange(::ad::physics::Speed const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::physics::Speed>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::physics::Speed>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::Speed)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::physics::Speed>::lowest(),
                  std::numeric_limits<::ad::physics::Speed>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange = (::ad::physics::Speed(-100.) <= input) && (input <= ::ad::physics::Speed(100.));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::physics::Speed)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::physics::Speed(-100.),
                    ::ad::physics::Speed(100.)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
