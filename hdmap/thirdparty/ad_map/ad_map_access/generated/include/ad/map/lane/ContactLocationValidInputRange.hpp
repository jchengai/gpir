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
#include "ad/map/lane/ContactLocation.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given ContactLocation is within valid input range
 *
 * \param[in] input the ContactLocation as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if ContactLocation is considered to be within the specified input range
 *
 * \note the specified input range is defined by the valid enum literals.
 */
inline bool withinValidInputRange(::ad::map::lane::ContactLocation const &input, bool const logErrors = true)
{
  bool inValidInputRange = (input == ::ad::map::lane::ContactLocation::INVALID)
    || (input == ::ad::map::lane::ContactLocation::UNKNOWN) || (input == ::ad::map::lane::ContactLocation::LEFT)
    || (input == ::ad::map::lane::ContactLocation::RIGHT) || (input == ::ad::map::lane::ContactLocation::SUCCESSOR)
    || (input == ::ad::map::lane::ContactLocation::PREDECESSOR) || (input == ::ad::map::lane::ContactLocation::OVERLAP);
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::lane::ContactLocation)>> {}, raw value: {} ",
                  input,
                  static_cast<int32_t>(input)); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
