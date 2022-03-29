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
#include "ad/map/point/Latitude.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given Latitude is within valid input range
 *
 * \param[in] input the Latitude as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if Latitude is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::map::point::Latitude>::lowest() <= \c input <=
 * std::numeric_limits<::ad::map::point::Latitude>::max()
 *       -90 <= \c input <= 90
 */
inline bool withinValidInputRange(::ad::map::point::Latitude const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::map::point::Latitude>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::map::point::Latitude>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::point::Latitude)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::map::point::Latitude>::lowest(),
                  std::numeric_limits<::ad::map::point::Latitude>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange = (::ad::map::point::Latitude(-90) <= input) && (input <= ::ad::map::point::Latitude(90));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::map::point::Latitude)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::map::point::Latitude(-90),
                    ::ad::map::point::Latitude(90)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
