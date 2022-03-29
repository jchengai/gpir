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

#include "ad/map/match/MapMatchedPositionType.hpp"
#include <stdexcept>

std::string toString(::ad::map::match::MapMatchedPositionType const e)
{
  switch (e)
  {
    case ::ad::map::match::MapMatchedPositionType::INVALID:
      return std::string("::ad::map::match::MapMatchedPositionType::INVALID"); // LCOV_EXCL_BR_LINE
    case ::ad::map::match::MapMatchedPositionType::UNKNOWN:
      return std::string("::ad::map::match::MapMatchedPositionType::UNKNOWN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::match::MapMatchedPositionType::LANE_IN:
      return std::string("::ad::map::match::MapMatchedPositionType::LANE_IN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::match::MapMatchedPositionType::LANE_LEFT:
      return std::string("::ad::map::match::MapMatchedPositionType::LANE_LEFT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::match::MapMatchedPositionType::LANE_RIGHT:
      return std::string("::ad::map::match::MapMatchedPositionType::LANE_RIGHT"); // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE"); // LCOV_EXCL_BR_LINE
  }
}

template <>::ad::map::match::MapMatchedPositionType fromString(std::string const &str)
{
  if (str == std::string("::ad::map::match::MapMatchedPositionType::INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::MapMatchedPositionType::INVALID;
  }
  if (str == std::string("INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::MapMatchedPositionType::INVALID;
  }
  if (str == std::string("::ad::map::match::MapMatchedPositionType::UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::MapMatchedPositionType::UNKNOWN;
  }
  if (str == std::string("UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::MapMatchedPositionType::UNKNOWN;
  }
  if (str == std::string("::ad::map::match::MapMatchedPositionType::LANE_IN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::MapMatchedPositionType::LANE_IN;
  }
  if (str == std::string("LANE_IN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::MapMatchedPositionType::LANE_IN;
  }
  if (str == std::string("::ad::map::match::MapMatchedPositionType::LANE_LEFT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::MapMatchedPositionType::LANE_LEFT;
  }
  if (str == std::string("LANE_LEFT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::MapMatchedPositionType::LANE_LEFT;
  }
  if (str == std::string("::ad::map::match::MapMatchedPositionType::LANE_RIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::MapMatchedPositionType::LANE_RIGHT;
  }
  if (str == std::string("LANE_RIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::MapMatchedPositionType::LANE_RIGHT;
  }
  throw std::out_of_range("Invalid enum literal"); // LCOV_EXCL_BR_LINE
}
