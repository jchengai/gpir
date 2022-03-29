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
#include "ad/map/lane/LaneId.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given LaneId is within valid input range
 *
 * \param[in] input the LaneId as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if LaneId is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::map::lane::LaneId>::lowest() <= \c input <=
 * std::numeric_limits<::ad::map::lane::LaneId>::max()
 *       1 <= \c input
 */
inline bool withinValidInputRange(::ad::map::lane::LaneId const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::map::lane::LaneId>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::map::lane::LaneId>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::lane::LaneId)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::map::lane::LaneId>::lowest(),
                  std::numeric_limits<::ad::map::lane::LaneId>::max()); // LCOV_EXCL_BR_LINE
  }
  // check for individual input range
  if (inValidInputRange)
  {
    inValidInputRange = (::ad::map::lane::LaneId(1) <= input);
    if (!inValidInputRange && logErrors)
    {
      spdlog::error("withinValidInputRange(::ad::map::lane::LaneId)>> {} out of valid input range [{}, {}]",
                    input,
                    ::ad::map::lane::LaneId(1),
                    "Undefined"); // LCOV_EXCL_BR_LINE
    }
  }
  return inValidInputRange;
}
