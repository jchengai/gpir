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
#include "ad/physics/Weight.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given Weight is within valid input range
 *
 * \param[in] input the Weight as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if Weight is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::physics::Weight>::lowest() <= \c input <=
 * std::numeric_limits<::ad::physics::Weight>::max()
 *       \c input <= 48600.0
 */
inline bool withinValidInputRange(::ad::physics::Weight const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::physics::Weight>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::physics::Weight>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::Weight)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::physics::Weight>::lowest(),
                  std::numeric_limits<::ad::physics::Weight>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange = (input <= ::ad::physics::Weight(48600.0));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::physics::Weight)>> {} out of valid input range [{}, {}]",
                    input,
                    "Undefined",
                    ::ad::physics::Weight(48600.0)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
