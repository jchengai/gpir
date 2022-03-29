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
#include "ad/map/intersection/IntersectionType.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given IntersectionType is within valid input range
 *
 * \param[in] input the IntersectionType as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if IntersectionType is considered to be within the specified input range
 *
 * \note the specified input range is defined by the valid enum literals.
 */
inline bool withinValidInputRange(::ad::map::intersection::IntersectionType const &input, bool const logErrors = true)
{
  bool inValidInputRange = (input == ::ad::map::intersection::IntersectionType::Unknown)
    || (input == ::ad::map::intersection::IntersectionType::Yield)
    || (input == ::ad::map::intersection::IntersectionType::Stop)
    || (input == ::ad::map::intersection::IntersectionType::AllWayStop)
    || (input == ::ad::map::intersection::IntersectionType::HasWay)
    || (input == ::ad::map::intersection::IntersectionType::Crosswalk)
    || (input == ::ad::map::intersection::IntersectionType::PriorityToRight)
    || (input == ::ad::map::intersection::IntersectionType::PriorityToRightAndStraight)
    || (input == ::ad::map::intersection::IntersectionType::TrafficLight);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::intersection::IntersectionType)>> {}, raw value: {} ",
                  input,
                  static_cast<int32_t>(input)); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
