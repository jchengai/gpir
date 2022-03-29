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

#include "ad/map/lane/LaneType.hpp"
#include <stdexcept>

std::string toString(::ad::map::lane::LaneType const e)
{
  switch (e)
  {
    case ::ad::map::lane::LaneType::INVALID:
      return std::string("::ad::map::lane::LaneType::INVALID"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneType::UNKNOWN:
      return std::string("::ad::map::lane::LaneType::UNKNOWN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneType::NORMAL:
      return std::string("::ad::map::lane::LaneType::NORMAL"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneType::INTERSECTION:
      return std::string("::ad::map::lane::LaneType::INTERSECTION"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneType::SHOULDER:
      return std::string("::ad::map::lane::LaneType::SHOULDER"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneType::EMERGENCY:
      return std::string("::ad::map::lane::LaneType::EMERGENCY"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneType::MULTI:
      return std::string("::ad::map::lane::LaneType::MULTI"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneType::PEDESTRIAN:
      return std::string("::ad::map::lane::LaneType::PEDESTRIAN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneType::OVERTAKING:
      return std::string("::ad::map::lane::LaneType::OVERTAKING"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneType::TURN:
      return std::string("::ad::map::lane::LaneType::TURN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneType::BIKE:
      return std::string("::ad::map::lane::LaneType::BIKE"); // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE"); // LCOV_EXCL_BR_LINE
  }
}

template <>::ad::map::lane::LaneType fromString(std::string const &str)
{
  if (str == std::string("::ad::map::lane::LaneType::INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::INVALID;
  }
  if (str == std::string("INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::INVALID;
  }
  if (str == std::string("::ad::map::lane::LaneType::UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::UNKNOWN;
  }
  if (str == std::string("UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::UNKNOWN;
  }
  if (str == std::string("::ad::map::lane::LaneType::NORMAL")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::NORMAL;
  }
  if (str == std::string("NORMAL")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::NORMAL;
  }
  if (str == std::string("::ad::map::lane::LaneType::INTERSECTION")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::INTERSECTION;
  }
  if (str == std::string("INTERSECTION")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::INTERSECTION;
  }
  if (str == std::string("::ad::map::lane::LaneType::SHOULDER")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::SHOULDER;
  }
  if (str == std::string("SHOULDER")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::SHOULDER;
  }
  if (str == std::string("::ad::map::lane::LaneType::EMERGENCY")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::EMERGENCY;
  }
  if (str == std::string("EMERGENCY")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::EMERGENCY;
  }
  if (str == std::string("::ad::map::lane::LaneType::MULTI")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::MULTI;
  }
  if (str == std::string("MULTI")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::MULTI;
  }
  if (str == std::string("::ad::map::lane::LaneType::PEDESTRIAN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::PEDESTRIAN;
  }
  if (str == std::string("PEDESTRIAN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::PEDESTRIAN;
  }
  if (str == std::string("::ad::map::lane::LaneType::OVERTAKING")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::OVERTAKING;
  }
  if (str == std::string("OVERTAKING")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::OVERTAKING;
  }
  if (str == std::string("::ad::map::lane::LaneType::TURN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::TURN;
  }
  if (str == std::string("TURN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::TURN;
  }
  if (str == std::string("::ad::map::lane::LaneType::BIKE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::BIKE;
  }
  if (str == std::string("BIKE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneType::BIKE;
  }
  throw std::out_of_range("Invalid enum literal"); // LCOV_EXCL_BR_LINE
}
