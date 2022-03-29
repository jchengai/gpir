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
#include "ad/map/match/ENUObjectPositionList.hpp"
#include "ad/map/match/ENUObjectPositionValidInputRange.hpp"
#include "spdlog/fmt/ostr.h"
#include "spdlog/spdlog.h"

/*!
 * \brief check if the given ENUObjectPositionList is within valid input range
 *
 * \param[in] input the ENUObjectPositionList as an input value
 * \param[in] logErrors enables error logging
 *
 * \returns \c true if ENUObjectPositionList is considered to be within the specified input range
 *
 * \note the specified input range is defined by
 *       1 <= \c input.size() <= 0
 *       and the ranges of all vector elements
 */
inline bool withinValidInputRange(::ad::map::match::ENUObjectPositionList const &input, bool const logErrors = true)
{
  bool inValidInputRange = ((std::size_t(1)) <= input.size());
  if (!inValidInputRange && logErrors)
  {
    spdlog::error("withinValidInputRange(::ad::map::match::ENUObjectPositionList)>> {}, invalid input range",
                  input); // LCOV_EXCL_BR_LINE
  }

  if (inValidInputRange)
  {
    for (auto const &member : input)
    {
      bool memberInValidInputRange = withinValidInputRange(member, logErrors);
      inValidInputRange = inValidInputRange && memberInValidInputRange;
      if (!memberInValidInputRange && logErrors)
      {
        spdlog::error("withinValidInputRange(::ad::map::match::ENUObjectPositionList)>> {}, invalid member {}",
                      input,
                      member); // LCOV_EXCL_BR_LINE
      }
    }
  }
  return inValidInputRange;
}
