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
#include "ad/map/match/MapMatchedPositionType.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given MapMatchedPositionType is within valid input range
 *
 * \param[in] input the MapMatchedPositionType as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if MapMatchedPositionType is considered to be within the specified input range
 *
 * \note the specified input range is defined by the valid enum literals.
 */
inline bool withinValidInputRange(::ad::map::match::MapMatchedPositionType const &input, bool const logErrors = true)
{
  bool inValidInputRange = (input == ::ad::map::match::MapMatchedPositionType::INVALID)
    || (input == ::ad::map::match::MapMatchedPositionType::UNKNOWN)
    || (input == ::ad::map::match::MapMatchedPositionType::LANE_IN)
    || (input == ::ad::map::match::MapMatchedPositionType::LANE_LEFT)
    || (input == ::ad::map::match::MapMatchedPositionType::LANE_RIGHT);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::match::MapMatchedPositionType)>> {}, raw value: {} ",
                  input,
                  static_cast<int32_t>(input)); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
