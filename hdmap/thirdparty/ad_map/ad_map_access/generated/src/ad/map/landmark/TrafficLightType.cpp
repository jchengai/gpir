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

#include "ad/map/landmark/TrafficLightType.hpp"
#include <stdexcept>

std::string toString(::ad::map::landmark::TrafficLightType const e)
{
  switch (e)
  {
    case ::ad::map::landmark::TrafficLightType::INVALID:
      return std::string("::ad::map::landmark::TrafficLightType::INVALID"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::UNKNOWN:
      return std::string("::ad::map::landmark::TrafficLightType::UNKNOWN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW:
      return std::string("::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN:
      return std::string("::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN:
      return std::string("::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN:
      return std::string("::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN:
      return std::string("::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN:
      return std::string("::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN:
      return std::string("::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN:
      return std::string("::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN:
      return std::string("::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN:
      return std::string("::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN:
      return std::string("::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN:
      return std::string("::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN:
      return std::string(
        "::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN"); // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE"); // LCOV_EXCL_BR_LINE
  }
}

template <>::ad::map::landmark::TrafficLightType fromString(std::string const &str)
{
  if (str == std::string("::ad::map::landmark::TrafficLightType::INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::INVALID;
  }
  if (str == std::string("INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::INVALID;
  }
  if (str == std::string("::ad::map::landmark::TrafficLightType::UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::UNKNOWN;
  }
  if (str == std::string("UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::UNKNOWN;
  }
  if (str == std::string("::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW;
  }
  if (str == std::string("SOLID_RED_YELLOW")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW;
  }
  if (str == std::string("::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN;
  }
  if (str == std::string("SOLID_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN;
  }
  if (str == std::string("::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN;
  }
  if (str == std::string("LEFT_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN;
  }
  if (str == std::string("::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN;
  }
  if (str == std::string("RIGHT_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN;
  }
  if (str == std::string("::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN;
  }
  if (str == std::string("STRAIGHT_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN;
  }
  if (str == std::string("::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN;
  }
  if (str == std::string("LEFT_STRAIGHT_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN;
  }
  if (str == std::string("::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN;
  }
  if (str == std::string("RIGHT_STRAIGHT_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN;
  }
  if (str == std::string("::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN;
  }
  if (str == std::string("PEDESTRIAN_RED_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_GREEN;
  }
  if (str == std::string("::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN;
  }
  if (str == std::string("BIKE_RED_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::BIKE_RED_GREEN;
  }
  if (str == std::string("::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN;
  }
  if (str == std::string("BIKE_PEDESTRIAN_RED_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN;
  }
  if (str == std::string("::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN;
  }
  if (str == std::string("PEDESTRIAN_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN;
  }
  if (str == std::string("::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN;
  }
  if (str == std::string("BIKE_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN;
  }
  if (str
      == std::string("::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN;
  }
  if (str == std::string("BIKE_PEDESTRIAN_RED_YELLOW_GREEN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN;
  }
  throw std::out_of_range("Invalid enum literal"); // LCOV_EXCL_BR_LINE
}
