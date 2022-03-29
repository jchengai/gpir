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
#include "ad/map/lane/ContactType.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given ContactType is within valid input range
 *
 * \param[in] input the ContactType as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if ContactType is considered to be within the specified input range
 *
 * \note the specified input range is defined by the valid enum literals.
 */
inline bool withinValidInputRange(::ad::map::lane::ContactType const &input, bool const logErrors = true)
{
  bool inValidInputRange = (input == ::ad::map::lane::ContactType::INVALID)
    || (input == ::ad::map::lane::ContactType::UNKNOWN) || (input == ::ad::map::lane::ContactType::FREE)
    || (input == ::ad::map::lane::ContactType::LANE_CHANGE)
    || (input == ::ad::map::lane::ContactType::LANE_CONTINUATION) || (input == ::ad::map::lane::ContactType::LANE_END)
    || (input == ::ad::map::lane::ContactType::SINGLE_POINT) || (input == ::ad::map::lane::ContactType::STOP)
    || (input == ::ad::map::lane::ContactType::STOP_ALL) || (input == ::ad::map::lane::ContactType::YIELD)
    || (input == ::ad::map::lane::ContactType::GATE_BARRIER) || (input == ::ad::map::lane::ContactType::GATE_TOLBOOTH)
    || (input == ::ad::map::lane::ContactType::GATE_SPIKES)
    || (input == ::ad::map::lane::ContactType::GATE_SPIKES_CONTRA) || (input == ::ad::map::lane::ContactType::CURB_UP)
    || (input == ::ad::map::lane::ContactType::CURB_DOWN) || (input == ::ad::map::lane::ContactType::SPEED_BUMP)
    || (input == ::ad::map::lane::ContactType::TRAFFIC_LIGHT) || (input == ::ad::map::lane::ContactType::CROSSWALK)
    || (input == ::ad::map::lane::ContactType::PRIO_TO_RIGHT) || (input == ::ad::map::lane::ContactType::RIGHT_OF_WAY)
    || (input == ::ad::map::lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::lane::ContactType)>> {}, raw value: {} ",
                  input,
                  static_cast<int32_t>(input)); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
