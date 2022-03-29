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

#include "ad/map/intersection/IntersectionType.hpp"
#include <stdexcept>

std::string toString(::ad::map::intersection::IntersectionType const e)
{
  switch (e)
  {
    case ::ad::map::intersection::IntersectionType::Unknown:
      return std::string("::ad::map::intersection::IntersectionType::Unknown"); // LCOV_EXCL_BR_LINE
    case ::ad::map::intersection::IntersectionType::Yield:
      return std::string("::ad::map::intersection::IntersectionType::Yield"); // LCOV_EXCL_BR_LINE
    case ::ad::map::intersection::IntersectionType::Stop:
      return std::string("::ad::map::intersection::IntersectionType::Stop"); // LCOV_EXCL_BR_LINE
    case ::ad::map::intersection::IntersectionType::AllWayStop:
      return std::string("::ad::map::intersection::IntersectionType::AllWayStop"); // LCOV_EXCL_BR_LINE
    case ::ad::map::intersection::IntersectionType::HasWay:
      return std::string("::ad::map::intersection::IntersectionType::HasWay"); // LCOV_EXCL_BR_LINE
    case ::ad::map::intersection::IntersectionType::Crosswalk:
      return std::string("::ad::map::intersection::IntersectionType::Crosswalk"); // LCOV_EXCL_BR_LINE
    case ::ad::map::intersection::IntersectionType::PriorityToRight:
      return std::string("::ad::map::intersection::IntersectionType::PriorityToRight"); // LCOV_EXCL_BR_LINE
    case ::ad::map::intersection::IntersectionType::PriorityToRightAndStraight:
      return std::string("::ad::map::intersection::IntersectionType::PriorityToRightAndStraight"); // LCOV_EXCL_BR_LINE
    case ::ad::map::intersection::IntersectionType::TrafficLight:
      return std::string("::ad::map::intersection::IntersectionType::TrafficLight"); // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE"); // LCOV_EXCL_BR_LINE
  }
}

template <>::ad::map::intersection::IntersectionType fromString(std::string const &str)
{
  if (str == std::string("::ad::map::intersection::IntersectionType::Unknown")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::Unknown;
  }
  if (str == std::string("Unknown")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::Unknown;
  }
  if (str == std::string("::ad::map::intersection::IntersectionType::Yield")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::Yield;
  }
  if (str == std::string("Yield")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::Yield;
  }
  if (str == std::string("::ad::map::intersection::IntersectionType::Stop")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::Stop;
  }
  if (str == std::string("Stop")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::Stop;
  }
  if (str == std::string("::ad::map::intersection::IntersectionType::AllWayStop")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::AllWayStop;
  }
  if (str == std::string("AllWayStop")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::AllWayStop;
  }
  if (str == std::string("::ad::map::intersection::IntersectionType::HasWay")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::HasWay;
  }
  if (str == std::string("HasWay")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::HasWay;
  }
  if (str == std::string("::ad::map::intersection::IntersectionType::Crosswalk")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::Crosswalk;
  }
  if (str == std::string("Crosswalk")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::Crosswalk;
  }
  if (str == std::string("::ad::map::intersection::IntersectionType::PriorityToRight")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::PriorityToRight;
  }
  if (str == std::string("PriorityToRight")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::PriorityToRight;
  }
  if (str == std::string("::ad::map::intersection::IntersectionType::PriorityToRightAndStraight")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::PriorityToRightAndStraight;
  }
  if (str == std::string("PriorityToRightAndStraight")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::PriorityToRightAndStraight;
  }
  if (str == std::string("::ad::map::intersection::IntersectionType::TrafficLight")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::TrafficLight;
  }
  if (str == std::string("TrafficLight")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::intersection::IntersectionType::TrafficLight;
  }
  throw std::out_of_range("Invalid enum literal"); // LCOV_EXCL_BR_LINE
}
