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
#include "ad/physics/DistanceValidInputRange.hpp"
#include "ad/physics/MetricRange.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given MetricRange is within valid input range
 *
 * \param[in] input the MetricRange as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if MetricRange is considered to be within the specified input range
 *
 * \note the specified input range is defined by the ranges of all members, plus:
 *       ::ad::physics::Distance(0.) <= minimum <= maximum
 *       minimum <= maximum <= ::ad::physics::Distance(1e6)
 */
inline bool withinValidInputRange(::ad::physics::MetricRange const &input, bool const logErrors = true)
{
  // check for generic member input ranges
  bool inValidInputRange = true;
  inValidInputRange
    = withinValidInputRange(input.minimum, logErrors) && withinValidInputRange(input.maximum, logErrors);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::physics::MetricRange)>> {} has invalid member",
                  input); // LCOV_EXCL_BR_LINE
  }

  // check for individual input ranges
  if (inValidInputRange)
  {
    inValidInputRange = (::ad::physics::Distance(0.) <= input.minimum) && (input.minimum <= input.maximum);
    if (!inValidInputRange && logErrors)
    {
      spdlog::error(
        "withinValidInputRange(::ad::physics::MetricRange)>> {} element {} out of valid input range [{}, {}]",
        input,
        input.minimum,
        ::ad::physics::Distance(0.),
        input.maximum); // LCOV_EXCL_BR_LINE
    }
  }

  if (inValidInputRange)
  {
    inValidInputRange = (input.minimum <= input.maximum) && (input.maximum <= ::ad::physics::Distance(1e6));
    if (!inValidInputRange && logErrors)
    {
      spdlog::error(
        "withinValidInputRange(::ad::physics::MetricRange)>> {} element {} out of valid input range [{}, {}]",
        input,
        input.maximum,
        input.minimum,
        ::ad::physics::Distance(1e6)); // LCOV_EXCL_BR_LINE
    }
  }

  return inValidInputRange;
}
