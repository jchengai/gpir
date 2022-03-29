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
#include "ad/map/point/AltitudeValidInputRange.hpp"
#include "ad/map/point/GeoPoint.hpp"
#include "ad/map/point/LatitudeValidInputRange.hpp"
#include "ad/map/point/LongitudeValidInputRange.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given GeoPoint is within valid input range
 *
 * \param[in] input the GeoPoint as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if GeoPoint is considered to be within the specified input range
 *
 * \note the specified input range is defined by the ranges of all members
 */
inline bool withinValidInputRange(::ad::map::point::GeoPoint const &input, bool const logErrors = true)
{
  // check for generic member input ranges
  bool inValidInputRange = true;
  inValidInputRange = withinValidInputRange(input.longitude, logErrors)
    && withinValidInputRange(input.latitude, logErrors) && withinValidInputRange(input.altitude, logErrors);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::point::GeoPoint)>> {} has invalid member",
                  input); // LCOV_EXCL_BR_LINE
  }

  return inValidInputRange;
}
