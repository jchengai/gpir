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

#include "ad/map/lane/LaneDirection.hpp"

#include <stdexcept>

std::string toString(::ad::map::lane::LaneDirection const e) {
  switch (e) {
    case ::ad::map::lane::LaneDirection::INVALID:
      return std::string("INVALID");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneDirection::UNKNOWN:
      return std::string("UNKNOWN");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneDirection::POSITIVE:
      return std::string("POSITIVE");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneDirection::NEGATIVE:
      return std::string("NEGATIVE");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneDirection::REVERSABLE:
      return std::string("REVERSABLE");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneDirection::BIDIRECTIONAL:
      return std::string("BIDIRECTIONAL");  // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::LaneDirection::NONE:
      return std::string("NONE");  // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE");  // LCOV_EXCL_BR_LINE
  }
}

template <>
::ad::map::lane::LaneDirection fromString(std::string const &str) {
  if (str ==
      std::string(
          "::ad::map::lane::LaneDirection::INVALID"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::INVALID;
  }
  if (str == std::string("INVALID"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::INVALID;
  }
  if (str ==
      std::string(
          "::ad::map::lane::LaneDirection::UNKNOWN"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::UNKNOWN;
  }
  if (str == std::string("UNKNOWN"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::UNKNOWN;
  }
  if (str ==
      std::string(
          "::ad::map::lane::LaneDirection::POSITIVE"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::POSITIVE;
  }
  if (str == std::string("POSITIVE"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::POSITIVE;
  }
  if (str ==
      std::string(
          "::ad::map::lane::LaneDirection::NEGATIVE"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::NEGATIVE;
  }
  if (str == std::string("NEGATIVE"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::NEGATIVE;
  }
  if (str ==
      std::string(
          "::ad::map::lane::LaneDirection::REVERSABLE"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::REVERSABLE;
  }
  if (str == std::string("REVERSABLE"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::REVERSABLE;
  }
  if (str ==
      std::string(
          "::ad::map::lane::LaneDirection::BIDIRECTIONAL"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::BIDIRECTIONAL;
  }
  if (str == std::string("BIDIRECTIONAL"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::BIDIRECTIONAL;
  }
  if (str ==
      std::string("::ad::map::lane::LaneDirection::NONE"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::NONE;
  }
  if (str == std::string("NONE"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::LaneDirection::NONE;
  }
  throw std::out_of_range("Invalid enum literal");  // LCOV_EXCL_BR_LINE
}
