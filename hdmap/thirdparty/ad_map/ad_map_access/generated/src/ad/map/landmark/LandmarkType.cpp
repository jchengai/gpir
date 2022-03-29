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

#include "ad/map/landmark/LandmarkType.hpp"
#include <stdexcept>

std::string toString(::ad::map::landmark::LandmarkType const e)
{
  switch (e)
  {
    case ::ad::map::landmark::LandmarkType::INVALID:
      return std::string("::ad::map::landmark::LandmarkType::INVALID"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::LandmarkType::UNKNOWN:
      return std::string("::ad::map::landmark::LandmarkType::UNKNOWN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::LandmarkType::TRAFFIC_SIGN:
      return std::string("::ad::map::landmark::LandmarkType::TRAFFIC_SIGN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT:
      return std::string("::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::LandmarkType::POLE:
      return std::string("::ad::map::landmark::LandmarkType::POLE"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::LandmarkType::GUIDE_POST:
      return std::string("::ad::map::landmark::LandmarkType::GUIDE_POST"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::LandmarkType::TREE:
      return std::string("::ad::map::landmark::LandmarkType::TREE"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::LandmarkType::STREET_LAMP:
      return std::string("::ad::map::landmark::LandmarkType::STREET_LAMP"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::LandmarkType::POSTBOX:
      return std::string("::ad::map::landmark::LandmarkType::POSTBOX"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::LandmarkType::MANHOLE:
      return std::string("::ad::map::landmark::LandmarkType::MANHOLE"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::LandmarkType::POWERCABINET:
      return std::string("::ad::map::landmark::LandmarkType::POWERCABINET"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::LandmarkType::FIRE_HYDRANT:
      return std::string("::ad::map::landmark::LandmarkType::FIRE_HYDRANT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::LandmarkType::BOLLARD:
      return std::string("::ad::map::landmark::LandmarkType::BOLLARD"); // LCOV_EXCL_BR_LINE
    case ::ad::map::landmark::LandmarkType::OTHER:
      return std::string("::ad::map::landmark::LandmarkType::OTHER"); // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE"); // LCOV_EXCL_BR_LINE
  }
}

template <>::ad::map::landmark::LandmarkType fromString(std::string const &str)
{
  if (str == std::string("::ad::map::landmark::LandmarkType::INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::INVALID;
  }
  if (str == std::string("INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::INVALID;
  }
  if (str == std::string("::ad::map::landmark::LandmarkType::UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::UNKNOWN;
  }
  if (str == std::string("UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::UNKNOWN;
  }
  if (str == std::string("::ad::map::landmark::LandmarkType::TRAFFIC_SIGN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::TRAFFIC_SIGN;
  }
  if (str == std::string("TRAFFIC_SIGN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::TRAFFIC_SIGN;
  }
  if (str == std::string("::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT;
  }
  if (str == std::string("TRAFFIC_LIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::TRAFFIC_LIGHT;
  }
  if (str == std::string("::ad::map::landmark::LandmarkType::POLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::POLE;
  }
  if (str == std::string("POLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::POLE;
  }
  if (str == std::string("::ad::map::landmark::LandmarkType::GUIDE_POST")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::GUIDE_POST;
  }
  if (str == std::string("GUIDE_POST")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::GUIDE_POST;
  }
  if (str == std::string("::ad::map::landmark::LandmarkType::TREE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::TREE;
  }
  if (str == std::string("TREE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::TREE;
  }
  if (str == std::string("::ad::map::landmark::LandmarkType::STREET_LAMP")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::STREET_LAMP;
  }
  if (str == std::string("STREET_LAMP")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::STREET_LAMP;
  }
  if (str == std::string("::ad::map::landmark::LandmarkType::POSTBOX")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::POSTBOX;
  }
  if (str == std::string("POSTBOX")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::POSTBOX;
  }
  if (str == std::string("::ad::map::landmark::LandmarkType::MANHOLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::MANHOLE;
  }
  if (str == std::string("MANHOLE")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::MANHOLE;
  }
  if (str == std::string("::ad::map::landmark::LandmarkType::POWERCABINET")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::POWERCABINET;
  }
  if (str == std::string("POWERCABINET")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::POWERCABINET;
  }
  if (str == std::string("::ad::map::landmark::LandmarkType::FIRE_HYDRANT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::FIRE_HYDRANT;
  }
  if (str == std::string("FIRE_HYDRANT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::FIRE_HYDRANT;
  }
  if (str == std::string("::ad::map::landmark::LandmarkType::BOLLARD")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::BOLLARD;
  }
  if (str == std::string("BOLLARD")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::BOLLARD;
  }
  if (str == std::string("::ad::map::landmark::LandmarkType::OTHER")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::OTHER;
  }
  if (str == std::string("OTHER")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::landmark::LandmarkType::OTHER;
  }
  throw std::out_of_range("Invalid enum literal"); // LCOV_EXCL_BR_LINE
}
