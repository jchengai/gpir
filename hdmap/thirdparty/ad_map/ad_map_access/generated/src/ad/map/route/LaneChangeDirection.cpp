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

#include "ad/map/route/LaneChangeDirection.hpp"
#include <stdexcept>

std::string toString(::ad::map::route::LaneChangeDirection const e)
{
  switch (e)
  {
    case ::ad::map::route::LaneChangeDirection::LeftToRight:
      return std::string("::ad::map::route::LaneChangeDirection::LeftToRight"); // LCOV_EXCL_BR_LINE
    case ::ad::map::route::LaneChangeDirection::RightToLeft:
      return std::string("::ad::map::route::LaneChangeDirection::RightToLeft"); // LCOV_EXCL_BR_LINE
    case ::ad::map::route::LaneChangeDirection::Invalid:
      return std::string("::ad::map::route::LaneChangeDirection::Invalid"); // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE"); // LCOV_EXCL_BR_LINE
  }
}

template <>::ad::map::route::LaneChangeDirection fromString(std::string const &str)
{
  if (str == std::string("::ad::map::route::LaneChangeDirection::LeftToRight")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::LaneChangeDirection::LeftToRight;
  }
  if (str == std::string("LeftToRight")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::LaneChangeDirection::LeftToRight;
  }
  if (str == std::string("::ad::map::route::LaneChangeDirection::RightToLeft")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::LaneChangeDirection::RightToLeft;
  }
  if (str == std::string("RightToLeft")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::LaneChangeDirection::RightToLeft;
  }
  if (str == std::string("::ad::map::route::LaneChangeDirection::Invalid")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::LaneChangeDirection::Invalid;
  }
  if (str == std::string("Invalid")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::LaneChangeDirection::Invalid;
  }
  throw std::out_of_range("Invalid enum literal"); // LCOV_EXCL_BR_LINE
}
