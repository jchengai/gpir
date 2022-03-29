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
#include "ad/physics/Duration.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given Duration is within valid input range
 *
 * \param[in] input the Duration as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if Duration is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::physics::Duration>::lowest() <= \c input <=
 * std::numeric_limits<::ad::physics::Duration>::max()
 *       0. <= \c input <= 1e6
 */
inline bool withinValidInputRange(::ad::physics::Duration const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::physics::Duration>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::physics::Duration>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::Duration)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::physics::Duration>::lowest(),
                  std::numeric_limits<::ad::physics::Duration>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange = (::ad::physics::Duration(0.) <= input) && (input <= ::ad::physics::Duration(1e6));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::physics::Duration)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::physics::Duration(0.),
                    ::ad::physics::Duration(1e6)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
