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

#include "ad/map/restriction/RoadUserType.hpp"
#include <stdexcept>

std::string toString(::ad::map::restriction::RoadUserType const e)
{
  switch (e)
  {
    case ::ad::map::restriction::RoadUserType::INVALID:
      return std::string("::ad::map::restriction::RoadUserType::INVALID"); // LCOV_EXCL_BR_LINE
    case ::ad::map::restriction::RoadUserType::UNKNOWN:
      return std::string("::ad::map::restriction::RoadUserType::UNKNOWN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::restriction::RoadUserType::CAR:
      return std::string("::ad::map::restriction::RoadUserType::CAR"); // LCOV_EXCL_BR_LINE
    case ::ad::map::restriction::RoadUserType::BUS:
      return std::string("::ad::map::restriction::RoadUserType::BUS"); // LCOV_EXCL_BR_LINE
    case ::ad::map::restriction::RoadUserType::TRUCK:
      return std::string("::ad::map::restriction::RoadUserType::TRUCK"); // LCOV_EXCL_BR_LINE
    case ::ad::map::restriction::RoadUserType::PEDESTRIAN:
      return std::string("::ad::map::restriction::RoadUserType::PEDESTRIAN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::restriction::RoadUserType::MOTORBIKE:
      return std::string("::ad::map::restriction::RoadUserType::MOTORBIKE"); // LCOV_EXCL_BR_LINE
    case ::ad::map::restriction::RoadUserType::BICYCLE:
      return std::string("::ad::map::restriction::RoadUserType::BICYCLE"); // LCOV_EXCL_BR_LINE
    case ::ad::map::restriction::RoadUserType::CAR_ELECTRIC:
      return std::string("::ad::map::restriction::RoadUserType::CAR_ELECTRIC"); // LCOV_EXCL_BR_LINE
    case ::ad::map::restriction::RoadUserType::CAR_HYBRID:
      return std::string("::ad::map::restriction::RoadUserType::CAR_HYBRID"); // LCOV_EXCL_BR_LINE
    case ::ad::map::restriction::RoadUserType::CAR_PETROL:
      return std::string("::ad::map::restriction::RoadUserType::CAR_PETROL"); // LCOV_EXCL_BR_LINE
    case ::ad::map::restriction::RoadUserType::CAR_DIESEL:
      return std::string("::ad::map::restriction::RoadUserType::CAR_DIESEL"); // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE"); // LCOV_EXCL_BR_LINE
  }
}

template <>::ad::map::restriction::RoadUserType fromString(std::string const &str)
{
  if (str == std::string("::ad::map::restriction::RoadUserType::INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::INVALID;
  }
  if (str == std::string("INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::INVALID;
  }
  if (str == std::string("::ad::map::restriction::RoadUserType::UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::UNKNOWN;
  }
  if (str == std::string("UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::UNKNOWN;
  }
  if (str == std::string("::ad::map::restriction::RoadUserType::CAR")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::CAR;
  }
  if (str == std::string("CAR")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::CAR;
  }
  if (str == std::string("::ad::map::restriction::RoadUserType::BUS")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::BUS;
  }
  if (str == std::string("BUS")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::BUS;
  }
  if (str == std::string("::ad::map::restriction::RoadUserType::TRUCK")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::TRUCK;
  }
  if (str == std::string("TRUCK")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::TRUCK;
  }
  if (str == std::string("::ad::map::restriction::RoadUserType::PEDESTRIAN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::PEDESTRIAN;
  }
  if (str == std::string("PEDESTRIAN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::PEDESTRIAN;
  }
  if (str == std::string("::ad::map::restriction::RoadUserType::MOTORBIKE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::MOTORBIKE;
  }
  if (str == std::string("MOTORBIKE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::MOTORBIKE;
  }
  if (str == std::string("::ad::map::restriction::RoadUserType::BICYCLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::BICYCLE;
  }
  if (str == std::string("BICYCLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::BICYCLE;
  }
  if (str == std::string("::ad::map::restriction::RoadUserType::CAR_ELECTRIC")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::CAR_ELECTRIC;
  }
  if (str == std::string("CAR_ELECTRIC")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::CAR_ELECTRIC;
  }
  if (str == std::string("::ad::map::restriction::RoadUserType::CAR_HYBRID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::CAR_HYBRID;
  }
  if (str == std::string("CAR_HYBRID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::CAR_HYBRID;
  }
  if (str == std::string("::ad::map::restriction::RoadUserType::CAR_PETROL")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::CAR_PETROL;
  }
  if (str == std::string("CAR_PETROL")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::CAR_PETROL;
  }
  if (str == std::string("::ad::map::restriction::RoadUserType::CAR_DIESEL")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::CAR_DIESEL;
  }
  if (str == std::string("CAR_DIESEL")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::restriction::RoadUserType::CAR_DIESEL;
  }
  throw std::out_of_range("Invalid enum literal"); // LCOV_EXCL_BR_LINE
}
