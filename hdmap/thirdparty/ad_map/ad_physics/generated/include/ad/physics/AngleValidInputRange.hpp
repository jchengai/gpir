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
#include "ad/physics/Angle.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given Angle is within valid input range
 *
 * \param[in] input the Angle as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if Angle is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::physics::Angle>::lowest() <= \c input <=
 * std::numeric_limits<::ad::physics::Angle>::max()
 *       -6.283185308 <= \c input <= 6.283185308
 */
inline bool withinValidInputRange(::ad::physics::Angle const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::physics::Angle>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::physics::Angle>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::Angle)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::physics::Angle>::lowest(),
                  std::numeric_limits<::ad::physics::Angle>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange = (::ad::physics::Angle(-6.283185308) <= input) && (input <= ::ad::physics::Angle(6.283185308));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::physics::Angle)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::physics::Angle(-6.283185308),
                    ::ad::physics::Angle(6.283185308)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
