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
#include "ad/map/point/Altitude.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given Altitude is within valid input range
 *
 * \param[in] input the Altitude as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if Altitude is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::map::point::Altitude>::lowest() <= \c input <=
 * std::numeric_limits<::ad::map::point::Altitude>::max()
 *       -11000 <= \c input <= 9000
 */
inline bool withinValidInputRange(::ad::map::point::Altitude const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::map::point::Altitude>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::map::point::Altitude>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::point::Altitude)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::map::point::Altitude>::lowest(),
                  std::numeric_limits<::ad::map::point::Altitude>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange = (::ad::map::point::Altitude(-11000) <= input) && (input <= ::ad::map::point::Altitude(9000));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::map::point::Altitude)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::map::point::Altitude(-11000),
                    ::ad::map::point::Altitude(9000)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
