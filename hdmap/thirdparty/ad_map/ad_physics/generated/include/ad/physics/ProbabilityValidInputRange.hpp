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
#include "ad/physics/Probability.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given Probability is within valid input range
 *
 * \param[in] input the Probability as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if Probability is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::physics::Probability>::lowest() <= \c input <=
 * std::numeric_limits<::ad::physics::Probability>::max()
 *       0. <= \c input <= 1.
 */
inline bool withinValidInputRange(::ad::physics::Probability const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::physics::Probability>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::physics::Probability>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::Probability)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::physics::Probability>::lowest(),
                  std::numeric_limits<::ad::physics::Probability>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange = (::ad::physics::Probability(0.) <= input) && (input <= ::ad::physics::Probability(1.));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::physics::Probability)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::physics::Probability(0.),
                    ::ad::physics::Probability(1.)); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
