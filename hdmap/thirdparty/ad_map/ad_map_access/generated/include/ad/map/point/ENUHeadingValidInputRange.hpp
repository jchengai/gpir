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
#include "ad/map/point/ENUHeading.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given ENUHeading is within valid input range
 *
 * \param[in] input the ENUHeading as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if ENUHeading is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::map::point::ENUHeading>::lowest() <= \c input <=
 * std::numeric_limits<::ad::map::point::ENUHeading>::max()
 *       -3.141592655 <= \c input <= 3.141592655
 */
inline bool withinValidInputRange(::ad::map::point::ENUHeading const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::map::point::ENUHeading>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::map::point::ENUHeading>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::point::ENUHeading)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::map::point::ENUHeading>::lowest(),
                  std::numeric_limits<::ad::map::point::ENUHeading>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange
      = (::ad::map::point::ENUHeading(-3.141592655) <= input) && (input <= ::ad::map::point::ENUHeading(3.141592655));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::map::point::ENUHeading)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::map::point::ENUHeading(-3.141592655),
                    ::ad::map::point::ENUHeading(3.141592655)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
