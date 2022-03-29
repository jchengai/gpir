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
#include "ad/map/landmark/LandmarkIdValidInputRange.hpp"
#include "ad/map/lane/ContactLane.hpp"
#include "ad/map/lane/ContactLocationValidInputRange.hpp"
#include "ad/map/lane/ContactTypeListValidInputRange.hpp"
#include "ad/map/lane/LaneIdValidInputRange.hpp"
#include "ad/map/restriction/RestrictionsValidInputRange.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given ContactLane is within valid input range
 *
 * \param[in] input the ContactLane as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if ContactLane is considered to be within the specified input range
 *
 * \note the specified input range is defined by the ranges of all members
 */
inline bool withinValidInputRange(::ad::map::lane::ContactLane const &input, bool const logErrors = true)
{
  // check for generic member input ranges
  bool inValidInputRange = true;
  inValidInputRange = withinValidInputRange(input.toLane, logErrors) && withinValidInputRange(input.location, logErrors)
    && withinValidInputRange(input.types, logErrors) && withinValidInputRange(input.restrictions, logErrors)
    && withinValidInputRange(input.trafficLightId, logErrors);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::lane::ContactLane)>> {} has invalid member",
                  input); // LCOV_EXCL_BR_LINE
  }

  return inValidInputRange;
}
