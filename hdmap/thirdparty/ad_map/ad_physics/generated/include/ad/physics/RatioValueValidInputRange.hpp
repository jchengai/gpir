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
#include "ad/physics/RatioValue.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given RatioValue is within valid input range
 *
 * \param[in] input the RatioValue as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if RatioValue is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::physics::RatioValue>::lowest() <= \c input <=
 * std::numeric_limits<::ad::physics::RatioValue>::max()
 */
inline bool withinValidInputRange(::ad::physics::RatioValue const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::physics::RatioValue>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::physics::RatioValue>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::RatioValue)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::physics::RatioValue>::lowest(),
                  std::numeric_limits<::ad::physics::RatioValue>::max()); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
