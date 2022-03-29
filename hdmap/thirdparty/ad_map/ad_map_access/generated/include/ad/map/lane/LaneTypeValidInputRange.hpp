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
#include "ad/map/lane/LaneType.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given LaneType is within valid input range
 *
 * \param[in] input the LaneType as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if LaneType is considered to be within the specified input range
 *
 * \note the specified input range is defined by the valid enum literals.
 */
inline bool withinValidInputRange(::ad::map::lane::LaneType const &input, bool const logErrors = true)
{
  bool inValidInputRange = (input == ::ad::map::lane::LaneType::INVALID)
    || (input == ::ad::map::lane::LaneType::UNKNOWN) || (input == ::ad::map::lane::LaneType::NORMAL)
    || (input == ::ad::map::lane::LaneType::INTERSECTION) || (input == ::ad::map::lane::LaneType::SHOULDER)
    || (input == ::ad::map::lane::LaneType::EMERGENCY) || (input == ::ad::map::lane::LaneType::MULTI)
    || (input == ::ad::map::lane::LaneType::PEDESTRIAN) || (input == ::ad::map::lane::LaneType::OVERTAKING)
    || (input == ::ad::map::lane::LaneType::TURN) || (input == ::ad::map::lane::LaneType::BIKE);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::lane::LaneType)>> {}, raw value: {} ",
                  input,
                  static_cast<int32_t>(input)); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
