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
#include "ad/map/route/ConnectingRoute.hpp"
#include "ad/map/route/ConnectingRouteTypeValidInputRange.hpp"
#include "ad/map/route/FullRouteValidInputRange.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given ConnectingRoute is within valid input range
 *
 * \param[in] input the ConnectingRoute as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if ConnectingRoute is considered to be within the specified input range
 *
 * \note the specified input range is defined by the ranges of all members
 */
inline bool withinValidInputRange(::ad::map::route::ConnectingRoute const &input, bool const logErrors = true)
{
  // check for generic member input ranges
  bool inValidInputRange = true;
  inValidInputRange = withinValidInputRange(input.type, logErrors) && withinValidInputRange(input.routeA, logErrors)
    && withinValidInputRange(input.routeB, logErrors);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::route::ConnectingRoute)>> {} has invalid member",
                  input); // LCOV_EXCL_BR_LINE
  }

  return inValidInputRange;
}
