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
#include "ad/map/access/PartitionId.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given PartitionId is within valid input range
 *
 * \param[in] input the PartitionId as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if PartitionId is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       std::numeric_limits<::ad::map::access::PartitionId>::lowest() <= \c input <=
 * std::numeric_limits<::ad::map::access::PartitionId>::max()
 */
inline bool withinValidInputRange(::ad::map::access::PartitionId const &input, bool const logErrors = true)
{
  // check for generic numeric limits of the type
  bool inValidInputRange = input.isValid() && (std::numeric_limits<::ad::map::access::PartitionId>::lowest() <= input)
    && (input <= std::numeric_limits<::ad::map::access::PartitionId>::max());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::access::PartitionId)>> {} out of numerical limits [{}, {}]",
                  input,
                  std::numeric_limits<::ad::map::access::PartitionId>::lowest(),
                  std::numeric_limits<::ad::map::access::PartitionId>::max()); // LCOV_EXCL_BR_LINE
  }
  return inValidInputRange;
}
