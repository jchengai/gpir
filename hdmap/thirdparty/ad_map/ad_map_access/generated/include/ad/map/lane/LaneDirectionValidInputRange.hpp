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
#include "ad/map/lane/LaneDirection.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given LaneDirection is within valid input range
 *
 * \param[in] input the LaneDirection as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if LaneDirection is considered to be within the specified input range
 *
 * \note the specified input range is defined by the valid enum literals.
 */
inline bool withinValidInputRange(::ad::map::lane::LaneDirection const &input, bool const logErrors = true)
{
  bool inValidInputRange = (input == ::ad::map::lane::LaneDirection::INVALID)
    || (input == ::ad::map::lane::LaneDirection::UNKNOWN) || (input == ::ad::map::lane::LaneDirection::POSITIVE)
    || (input == ::ad::map::lane::LaneDirection::NEGATIVE) || (input == ::ad::map::lane::LaneDirection::REVERSABLE)
    || (input == ::ad::map::lane::LaneDirection::BIDIRECTIONAL) || (input == ::ad::map::lane::LaneDirection::NONE);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::lane::LaneDirection)>> {}, raw value: {} ",
                  input,
                  static_cast<int32_t>(input)); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
