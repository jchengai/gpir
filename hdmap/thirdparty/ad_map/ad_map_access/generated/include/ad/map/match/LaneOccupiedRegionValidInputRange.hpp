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
#include "ad/map/lane/LaneIdValidInputRange.hpp"
#include "ad/map/match/LaneOccupiedRegion.hpp"
#include "ad/physics/ParametricRangeValidInputRange.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given LaneOccupiedRegion is within valid input range
 *
 * \param[in] input the LaneOccupiedRegion as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if LaneOccupiedRegion is considered to be within the specified input range
 *
 * \note the specified input range is defined by the ranges of all members
 */
inline bool withinValidInputRange(::ad::map::match::LaneOccupiedRegion const &input, bool const logErrors = true)
{
  // check for generic member input ranges
  bool inValidInputRange = true;
  inValidInputRange = withinValidInputRange(input.laneId, logErrors)
    && withinValidInputRange(input.longitudinalRange, logErrors)
    && withinValidInputRange(input.lateralRange, logErrors);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::match::LaneOccupiedRegion)>> {} has invalid member",
                  input); // LCOV_EXCL_BR_LINE
  }

  return inValidInputRange;
}
