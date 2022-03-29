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
#include "ad/map/lane/LaneIdListValidInputRange.hpp"
#include "ad/map/lane/LaneIdValidInputRange.hpp"
#include "ad/map/route/LaneIntervalValidInputRange.hpp"
#include "ad/map/route/LaneSegment.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given LaneSegment is within valid input range
 *
 * \param[in] input the LaneSegment as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if LaneSegment is considered to be within the specified input range
 *
 * \note the specified input range is defined by the ranges of all members
 */
inline bool withinValidInputRange(::ad::map::route::LaneSegment const &input, bool const logErrors = true)
{
  // check for generic member input ranges
  bool inValidInputRange = true;
  inValidInputRange = withinValidInputRange(input.leftNeighbor, logErrors)
    && withinValidInputRange(input.rightNeighbor, logErrors) && withinValidInputRange(input.predecessors, logErrors)
    && withinValidInputRange(input.successors, logErrors) && withinValidInputRange(input.laneInterval, logErrors);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::route::LaneSegment)>> {} has invalid member",
                  input); // LCOV_EXCL_BR_LINE
  }

  return inValidInputRange;
}
