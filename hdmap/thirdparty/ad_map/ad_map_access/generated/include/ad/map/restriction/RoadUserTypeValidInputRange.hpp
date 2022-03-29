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
#include "ad/map/restriction/RoadUserType.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given RoadUserType is within valid input range
 *
 * \param[in] input the RoadUserType as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if RoadUserType is considered to be within the specified input range
 *
 * \note the specified input range is defined by the valid enum literals.
 */
inline bool withinValidInputRange(::ad::map::restriction::RoadUserType const &input, bool const logErrors = true)
{
  bool inValidInputRange = (input == ::ad::map::restriction::RoadUserType::INVALID)
    || (input == ::ad::map::restriction::RoadUserType::UNKNOWN) || (input == ::ad::map::restriction::RoadUserType::CAR)
    || (input == ::ad::map::restriction::RoadUserType::BUS) || (input == ::ad::map::restriction::RoadUserType::TRUCK)
    || (input == ::ad::map::restriction::RoadUserType::PEDESTRIAN)
    || (input == ::ad::map::restriction::RoadUserType::MOTORBIKE)
    || (input == ::ad::map::restriction::RoadUserType::BICYCLE)
    || (input == ::ad::map::restriction::RoadUserType::CAR_ELECTRIC)
    || (input == ::ad::map::restriction::RoadUserType::CAR_HYBRID)
    || (input == ::ad::map::restriction::RoadUserType::CAR_PETROL)
    || (input == ::ad::map::restriction::RoadUserType::CAR_DIESEL);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::restriction::RoadUserType)>> {}, raw value: {} ",
                  input,
                  static_cast<int32_t>(input)); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
