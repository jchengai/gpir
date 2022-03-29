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
#include "ad/map/landmark/ENULandmark.hpp"
#include "ad/map/landmark/LandmarkIdValidInputRange.hpp"
#include "ad/map/landmark/LandmarkTypeValidInputRange.hpp"
#include "ad/map/landmark/TrafficLightTypeValidInputRange.hpp"
#include "ad/map/point/ENUHeadingValidInputRange.hpp"
#include "ad/map/point/ENUPointValidInputRange.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given ENULandmark is within valid input range
 *
 * \param[in] input the ENULandmark as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if ENULandmark is considered to be within the specified input range
 *
 * \note the specified input range is defined by the ranges of all members
 */
inline bool withinValidInputRange(::ad::map::landmark::ENULandmark const &input, bool const logErrors = true)
{
  // check for generic member input ranges
  bool inValidInputRange = true;
  inValidInputRange = withinValidInputRange(input.id, logErrors) && withinValidInputRange(input.type, logErrors)
    && withinValidInputRange(input.position, logErrors) && withinValidInputRange(input.heading, logErrors)
    && withinValidInputRange(input.trafficLightType, logErrors);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::landmark::ENULandmark)>> {} has invalid member",
                  input); // LCOV_EXCL_BR_LINE
  }

  return inValidInputRange;
}
