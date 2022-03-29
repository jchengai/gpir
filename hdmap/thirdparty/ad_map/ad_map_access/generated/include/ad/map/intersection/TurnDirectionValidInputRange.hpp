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
#include "ad/map/intersection/TurnDirection.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given TurnDirection is within valid input range
 *
 * \param[in] input the TurnDirection as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if TurnDirection is considered to be within the specified input range
 *
 * \note the specified input range is defined by the valid enum literals.
 */
inline bool withinValidInputRange(::ad::map::intersection::TurnDirection const &input, bool const logErrors = true)
{
  bool inValidInputRange = (input == ::ad::map::intersection::TurnDirection::Unknown)
    || (input == ::ad::map::intersection::TurnDirection::Right)
    || (input == ::ad::map::intersection::TurnDirection::Straight)
    || (input == ::ad::map::intersection::TurnDirection::Left)
    || (input == ::ad::map::intersection::TurnDirection::UTurn);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::intersection::TurnDirection)>> {}, raw value: {} ",
                  input,
                  static_cast<int32_t>(input)); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
