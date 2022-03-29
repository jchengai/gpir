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
#include "ad/physics/DurationSquared.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given DurationSquared is within valid input range
 *
 * \param[in] input the DurationSquared as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if DurationSquared is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::physics::DurationSquared>::lowest() <= \c input <=
 * std::numeric_limits<::ad::physics::DurationSquared>::max()
 *       0. <= \c input <= 10000.
 */
inline bool withinValidInputRange(::ad::physics::DurationSquared const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::physics::DurationSquared>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::physics::DurationSquared>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::DurationSquared)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::physics::DurationSquared>::lowest(),
                  std::numeric_limits<::ad::physics::DurationSquared>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange
      = (::ad::physics::DurationSquared(0.) <= input) && (input <= ::ad::physics::DurationSquared(10000.));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::physics::DurationSquared)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::physics::DurationSquared(0.),
                    ::ad::physics::DurationSquared(10000.)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
