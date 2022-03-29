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

#include "ad/map/access/TrafficType.hpp"
#include <stdexcept>

std::string toString(::ad::map::access::TrafficType const e)
{
  switch (e)
  {
    case ::ad::map::access::TrafficType::INVALID:
      return std::string("::ad::map::access::TrafficType::INVALID"); // LCOV_EXCL_BR_LINE
    case ::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC:
      return std::string("::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC"); // LCOV_EXCL_BR_LINE
    case ::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC:
      return std::string("::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC"); // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE"); // LCOV_EXCL_BR_LINE
  }
}

template <>::ad::map::access::TrafficType fromString(std::string const &str)
{
  if (str == std::string("::ad::map::access::TrafficType::INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::access::TrafficType::INVALID;
  }
  if (str == std::string("INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::access::TrafficType::INVALID;
  }
  if (str == std::string("::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC;
  }
  if (str == std::string("LEFT_HAND_TRAFFIC")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::access::TrafficType::LEFT_HAND_TRAFFIC;
  }
  if (str == std::string("::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC;
  }
  if (str == std::string("RIGHT_HAND_TRAFFIC")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::access::TrafficType::RIGHT_HAND_TRAFFIC;
  }
  throw std::out_of_range("Invalid enum literal"); // LCOV_EXCL_BR_LINE
}
