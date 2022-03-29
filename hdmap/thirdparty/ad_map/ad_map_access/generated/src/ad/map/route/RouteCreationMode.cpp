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

#include "ad/map/route/RouteCreationMode.hpp"
#include <stdexcept>

std::string toString(::ad::map::route::RouteCreationMode const e)
{
  switch (e)
  {
    case ::ad::map::route::RouteCreationMode::Undefined:
      return std::string("::ad::map::route::RouteCreationMode::Undefined"); // LCOV_EXCL_BR_LINE
    case ::ad::map::route::RouteCreationMode::SameDrivingDirection:
      return std::string("::ad::map::route::RouteCreationMode::SameDrivingDirection"); // LCOV_EXCL_BR_LINE
    case ::ad::map::route::RouteCreationMode::AllRoutableLanes:
      return std::string("::ad::map::route::RouteCreationMode::AllRoutableLanes"); // LCOV_EXCL_BR_LINE
    case ::ad::map::route::RouteCreationMode::AllNeighborLanes:
      return std::string("::ad::map::route::RouteCreationMode::AllNeighborLanes"); // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE"); // LCOV_EXCL_BR_LINE
  }
}

template <>::ad::map::route::RouteCreationMode fromString(std::string const &str)
{
  if (str == std::string("::ad::map::route::RouteCreationMode::Undefined")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::RouteCreationMode::Undefined;
  }
  if (str == std::string("Undefined")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::RouteCreationMode::Undefined;
  }
  if (str == std::string("::ad::map::route::RouteCreationMode::SameDrivingDirection")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::RouteCreationMode::SameDrivingDirection;
  }
  if (str == std::string("SameDrivingDirection")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::RouteCreationMode::SameDrivingDirection;
  }
  if (str == std::string("::ad::map::route::RouteCreationMode::AllRoutableLanes")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::RouteCreationMode::AllRoutableLanes;
  }
  if (str == std::string("AllRoutableLanes")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::RouteCreationMode::AllRoutableLanes;
  }
  if (str == std::string("::ad::map::route::RouteCreationMode::AllNeighborLanes")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::RouteCreationMode::AllNeighborLanes;
  }
  if (str == std::string("AllNeighborLanes")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::route::RouteCreationMode::AllNeighborLanes;
  }
  throw std::out_of_range("Invalid enum literal"); // LCOV_EXCL_BR_LINE
}
