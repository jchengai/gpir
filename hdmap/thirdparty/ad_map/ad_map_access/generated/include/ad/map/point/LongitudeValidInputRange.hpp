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
#include "ad/map/point/Longitude.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given Longitude is within valid input range
 *
 * \param[in] input the Longitude as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if Longitude is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::map::point::Longitude>::lowest() <= \c input <=
 * std::numeric_limits<::ad::map::point::Longitude>::max()
 *       -180 <= \c input <= 180
 */
inline bool withinValidInputRange(::ad::map::point::Longitude const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::map::point::Longitude>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::map::point::Longitude>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::point::Longitude)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::map::point::Longitude>::lowest(),
                  std::numeric_limits<::ad::map::point::Longitude>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange = (::ad::map::point::Longitude(-180) <= input) && (input <= ::ad::map::point::Longitude(180));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::map::point::Longitude)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::map::point::Longitude(-180),
                    ::ad::map::point::Longitude(180)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
