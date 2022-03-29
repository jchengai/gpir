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
#include "ad/physics/Distance.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given Distance is within valid input range
 *
 * \param[in] input the Distance as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if Distance is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::physics::Distance>::lowest() <= \c input <=
 * std::numeric_limits<::ad::physics::Distance>::max()
 *       -1e9 <= \c input <= 1e9
 */
inline bool withinValidInputRange(::ad::physics::Distance const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::physics::Distance>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::physics::Distance>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::Distance)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::physics::Distance>::lowest(),
                  std::numeric_limits<::ad::physics::Distance>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange = (::ad::physics::Distance(-1e9) <= input) && (input <= ::ad::physics::Distance(1e9));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::physics::Distance)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::physics::Distance(-1e9),
                    ::ad::physics::Distance(1e9)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
