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

#include "ad/map/route/ConnectingRouteType.hpp"
#include <stdexcept>

std::string toString(::ad::map::route::ConnectingRouteType const e)
{
  switch (e)
  {
    case ::ad::map::route::ConnectingRouteType::Invalid:
      return std::string("::ad::map::route::ConnectingRouteType::Invalid"); // LCOV_EXCL_BR_LINE
    case ::ad::map::route::ConnectingRouteType::Following:
      return std::string("::ad::map::route::ConnectingRouteType::Following"); // LCOV_EXCL_BR_LINE
    case ::ad::map::route::ConnectingRouteType::Opposing:
      return std::string("::ad::map::route::ConnectingRouteType::Opposing"); // LCOV_EXCL_BR_LINE
    case ::ad::map::route::ConnectingRouteType::Merging:
      return std::string("::ad::map::route::ConnectingRouteType::Merging"); // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE"); // LCOV_EXCL_BR_LINE
  }
}

template <>::ad::map::route::ConnectingRouteType fromString(std::string const &str)
{
  if (str == std::string("::ad::map::route::ConnectingRouteType::Invalid")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::ConnectingRouteType::Invalid;
  }
  if (str == std::string("Invalid")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::ConnectingRouteType::Invalid;
  }
  if (str == std::string("::ad::map::route::ConnectingRouteType::Following")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::ConnectingRouteType::Following;
  }
  if (str == std::string("Following")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::ConnectingRouteType::Following;
  }
  if (str == std::string("::ad::map::route::ConnectingRouteType::Opposing")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::ConnectingRouteType::Opposing;
  }
  if (str == std::string("Opposing")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::ConnectingRouteType::Opposing;
  }
  if (str == std::string("::ad::map::route::ConnectingRouteType::Merging")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::ConnectingRouteType::Merging;
  }
  if (str == std::string("Merging")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::ConnectingRouteType::Merging;
  }
  throw std::out_of_range("Invalid enum literal"); // LCOV_EXCL_BR_LINE
}
