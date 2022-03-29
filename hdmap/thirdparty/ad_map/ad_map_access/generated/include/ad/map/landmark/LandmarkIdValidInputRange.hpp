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
#include "ad/map/landmark/LandmarkId.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given LandmarkId is within valid input range
 *
 * \param[in] input the LandmarkId as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if LandmarkId is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest() <= \c input <=
 * std::numeric_limits<::ad::map::landmark::LandmarkId>::max()
 */
inline bool withinValidInputRange(::ad::map::landmark::LandmarkId const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::map::landmark::LandmarkId>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::landmark::LandmarkId)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::map::landmark::LandmarkId>::lowest(),
                  std::numeric_limits<::ad::map::landmark::LandmarkId>::max()); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
