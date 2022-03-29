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
#include "ad/map/landmark/TrafficLightType.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given TrafficLightType is within valid input range
 *
 * \param[in] input the TrafficLightType as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if TrafficLightType is considered to be within the specified input range
 *
 * \note the specified input range is defined by the valid enum literals.
 */
inline bool withinValidInputRange(::ad::map::landmark::TrafficLightType const &input, bool const logErrors = true)
{
  bool inValidInputRange = (input == ::ad::map::landmark::TrafficLightType::INVALID)
    || (input == ::ad::map::landmark::TrafficLightType::UNKNOWN)
    || (input == ::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW)
    || (input == ::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN)
    || (input == ::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN)
    || (input == ::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN)
    || (input == ::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN)
    || (input == ::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN)
    || (input == ::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN)
    || (input == ::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN)
    || (input == ::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN)
    || (input == ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN)
    || (input == ::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN)
    || (input == ::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN)
    || (input == ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::landmark::TrafficLightType)>> {}, raw value: {} ",
                  input,
                  static_cast<int32_t>(input)); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
