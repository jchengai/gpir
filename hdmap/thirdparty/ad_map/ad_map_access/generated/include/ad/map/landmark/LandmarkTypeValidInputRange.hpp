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
#include "ad/map/landmark/LandmarkType.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given LandmarkType is within valid input range
 *
 * \param[in] input the LandmarkType as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if LandmarkType is considered to be within the specified input range
 *
 * \note the specified input range is defined by the valid enum literals.
 */
inline bool withinValidInputRange(::ad::map::landmark::LandmarkType const &input, bool const logErrors = true)
{
  bool inValidInputRange = (input == ::ad::map::landmark::LandmarkType::INVALID)
    || (input == ::ad::map::landmark::LandmarkType::UNKNOWN)
    || (input == ::ad::map::landmark::LandmarkType::TRAFFIC_SIGN)
    || (input == ::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT) || (input == ::ad::map::landmark::LandmarkType::POLE)
    || (input == ::ad::map::landmark::LandmarkType::GUIDE_POST) || (input == ::ad::map::landmark::LandmarkType::TREE)
    || (input == ::ad::map::landmark::LandmarkType::STREET_LAMP)
    || (input == ::ad::map::landmark::LandmarkType::POSTBOX) || (input == ::ad::map::landmark::LandmarkType::MANHOLE)
    || (input == ::ad::map::landmark::LandmarkType::POWERCABINET)
    || (input == ::ad::map::landmark::LandmarkType::FIRE_HYDRANT)
    || (input == ::ad::map::landmark::LandmarkType::BOLLARD) || (input == ::ad::map::landmark::LandmarkType::OTHER);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::landmark::LandmarkType)>> {}, raw value: {} ",
                  input,
                  static_cast<int32_t>(input)); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
