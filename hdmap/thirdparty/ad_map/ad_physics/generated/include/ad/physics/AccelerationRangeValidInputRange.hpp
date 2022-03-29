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
#include "ad/physics/AccelerationRange.hpp"
#include "ad/physics/AccelerationValidInputRange.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given AccelerationRange is within valid input range
 *
 * \param[in] input the AccelerationRange as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if AccelerationRange is considered to be within the specified input range
 *
 * \note the specified input range is defined by the ranges of all members, plus:
 *       ::ad::physics::Acceleration(-1e2) <= minimum <= maximum
 *       minimum <= maximum <= ::ad::physics::Acceleration(1e2)
 */
inline bool withinValidInputRange(::ad::physics::AccelerationRange const &input, bool const logErrors = true)
{
  // check for generic member input ranges
  bool inValidInputRange = true;
  inValidInputRange
    = withinValidInputRange(input.minimum, logErrors) && withinValidInputRange(input.maximum, logErrors);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::AccelerationRange)>> {} has invalid member",
                  input); // LCOV_EXCL_BR_LINE
  }

  // check for individual input ranges
  if (inValidInputRange)
  {
    inValidInputRange = (::ad::physics::Acceleration(-1e2) <= input.minimum) && (input.minimum <= input.maximum);
    if (!inValidInputRange && logErrors)
    {
      spdlog::error(
        "withinValidInputRange(::ad::physics::AccelerationRange)>> {} element {} out of valid input range [{}, {}]",
        input,
        input.minimum,
        ::ad::physics::Acceleration(-1e2),
        input.maximum); // LCOV_EXCL_BR_LINE
    }
  }

  if (inValidInputRange)
  {
    inValidInputRange = (input.minimum <= input.maximum) && (input.maximum <= ::ad::physics::Acceleration(1e2));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error(
        "withinValidInputRange(::ad::physics::AccelerationRange)>> {} element {} out of valid input range [{}, {}]",
        input,
        input.maximum,
        input.minimum,
        ::ad::physics::Acceleration(1e2)); // LCOV_EXCL_BR_LINE
    }
  }

  return inValidInputRange;
}
