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
#include "ad/map/landmark/Landmark.hpp"
#include "ad/map/landmark/LandmarkIdValidInputRange.hpp"
#include "ad/map/landmark/LandmarkTypeValidInputRange.hpp"
#include "ad/map/landmark/TrafficLightTypeValidInputRange.hpp"
#include "ad/map/landmark/TrafficSignTypeValidInputRange.hpp"
#include "ad/map/point/ECEFPointValidInputRange.hpp"
#include "ad/map/point/GeometryValidInputRange.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given Landmark is within valid input range
 *
 * \param[in] input the Landmark as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if Landmark is considered to be within the specified input range
 *
 * \note the specified input range is defined by the ranges of all members
 */
inline bool withinValidInputRange(::ad::map::landmark::Landmark const &input, bool const logErrors = true)
{
  // check for generic member input ranges
  bool inValidInputRange = true;
  inValidInputRange = withinValidInputRange(input.id, logErrors) && withinValidInputRange(input.type, logErrors)
    && withinValidInputRange(input.position, logErrors) && withinValidInputRange(input.orientation, logErrors)
    && withinValidInputRange(input.boundingBox, logErrors) && withinValidInputRange(input.trafficLightType, logErrors)
    && withinValidInputRange(input.trafficSignType, logErrors);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::landmark::Landmark)>> {} has invalid member",
                  input); // LCOV_EXCL_BR_LINE
  }

  return inValidInputRange;
}
