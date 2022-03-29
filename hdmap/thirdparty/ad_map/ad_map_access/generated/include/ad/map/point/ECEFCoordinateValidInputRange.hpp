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
#include "ad/map/point/ECEFCoordinate.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given ECEFCoordinate is within valid input range
 *
 * \param[in] input the ECEFCoordinate as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if ECEFCoordinate is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::map::point::ECEFCoordinate>::lowest() <= \c input <=
 * std::numeric_limits<::ad::map::point::ECEFCoordinate>::max()
 *       -6400000 <= \c input <= 6400000
 */
inline bool withinValidInputRange(::ad::map::point::ECEFCoordinate const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::map::point::ECEFCoordinate>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::map::point::ECEFCoordinate>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::point::ECEFCoordinate)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::map::point::ECEFCoordinate>::lowest(),
                  std::numeric_limits<::ad::map::point::ECEFCoordinate>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange
      = (::ad::map::point::ECEFCoordinate(-6400000) <= input) && (input <= ::ad::map::point::ECEFCoordinate(6400000));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::map::point::ECEFCoordinate)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::map::point::ECEFCoordinate(-6400000),
                    ::ad::map::point::ECEFCoordinate(6400000)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
