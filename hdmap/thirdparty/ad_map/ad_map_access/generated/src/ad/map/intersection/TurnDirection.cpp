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

#include "ad/map/intersection/TurnDirection.hpp"

#include <stdexcept>

std::string toString(::ad::map::intersection::TurnDirection const e) {
  switch (e) {
    case ::ad::map::intersection::TurnDirection::Unknown:
      return std::string(
          "::ad::map::intersection::TurnDirection::Unknown");  // LCOV_EXCL_BR_LINE
    case ::ad::map::intersection::TurnDirection::Right:
      return std::string(
          "::ad::map::intersection::TurnDirection::Right");  // LCOV_EXCL_BR_LINE
    case ::ad::map::intersection::TurnDirection::Straight:
      return std::string(
          "::ad::map::intersection::TurnDirection::Straight");  // LCOV_EXCL_BR_LINE
    case ::ad::map::intersection::TurnDirection::Left:
      return std::string(
          "::ad::map::intersection::TurnDirection::Left");  // LCOV_EXCL_BR_LINE
    case ::ad::map::intersection::TurnDirection::UTurn:
      return std::string(
          "::ad::map::intersection::TurnDirection::UTurn");  // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE");  // LCOV_EXCL_BR_LINE
  }
}

template <>
::ad::map::intersection::TurnDirection fromString(std::string const &str) {
  if (str ==
      std::string(
          "::ad::map::intersection::TurnDirection::Unknown"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::TurnDirection::Unknown;
  }
  if (str == std::string("Unknown"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::TurnDirection::Unknown;
  }
  if (str ==
      std::string(
          "::ad::map::intersection::TurnDirection::Right"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::TurnDirection::Right;
  }
  if (str == std::string("Right"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::TurnDirection::Right;
  }
  if (str ==
      std::string(
          "::ad::map::intersection::TurnDirection::Straight"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::TurnDirection::Straight;
  }
  if (str == std::string("Straight"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::TurnDirection::Straight;
  }
  if (str ==
      std::string(
          "::ad::map::intersection::TurnDirection::Left"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::TurnDirection::Left;
  }
  if (str == std::string("Left"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::TurnDirection::Left;
  }
  if (str ==
      std::string(
          "::ad::map::intersection::TurnDirection::UTurn"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::TurnDirection::UTurn;
  }
  if (str == std::string("UTurn"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::TurnDirection::UTurn;
  }
  throw std::out_of_range("Invalid enum literal");  // LCOV_EXCL_BR_LINE
}
