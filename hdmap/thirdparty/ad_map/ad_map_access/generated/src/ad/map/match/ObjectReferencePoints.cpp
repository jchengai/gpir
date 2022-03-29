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

#include "ad/map/match/ObjectReferencePoints.hpp"

#include <stdexcept>

std::string toString(::ad::map::match::ObjectReferencePoints const e) {
  switch (e) {
    case ::ad::map::match::ObjectReferencePoints::FrontLeft:
      return std::string(
          "::ad::map::match::ObjectReferencePoints::FrontLeft");  // LCOV_EXCL_BR_LINE
    case ::ad::map::match::ObjectReferencePoints::FrontRight:
      return std::string(
          "::ad::map::match::ObjectReferencePoints::FrontRight");  // LCOV_EXCL_BR_LINE
    case ::ad::map::match::ObjectReferencePoints::RearLeft:
      return std::string(
          "::ad::map::match::ObjectReferencePoints::RearLeft");  // LCOV_EXCL_BR_LINE
    case ::ad::map::match::ObjectReferencePoints::RearRight:
      return std::string(
          "::ad::map::match::ObjectReferencePoints::RearRight");  // LCOV_EXCL_BR_LINE
    case ::ad::map::match::ObjectReferencePoints::Center:
      return std::string(
          "::ad::map::match::ObjectReferencePoints::Center");  // LCOV_EXCL_BR_LINE
    case ::ad::map::match::ObjectReferencePoints::NumPoints:
      return std::string(
          "::ad::map::match::ObjectReferencePoints::NumPoints");  // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE");  // LCOV_EXCL_BR_LINE
  }
}

template <>
::ad::map::match::ObjectReferencePoints fromString(std::string const &str) {
  if (str ==
      std::string(
          "::ad::map::match::ObjectReferencePoints::FrontLeft"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::ObjectReferencePoints::FrontLeft;
  }
  if (str == std::string("FrontLeft"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::ObjectReferencePoints::FrontLeft;
  }
  if (str ==
      std::string(
          "::ad::map::match::ObjectReferencePoints::FrontRight"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::ObjectReferencePoints::FrontRight;
  }
  if (str == std::string("FrontRight"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::ObjectReferencePoints::FrontRight;
  }
  if (str ==
      std::string(
          "::ad::map::match::ObjectReferencePoints::RearLeft"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::ObjectReferencePoints::RearLeft;
  }
  if (str == std::string("RearLeft"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::ObjectReferencePoints::RearLeft;
  }
  if (str ==
      std::string(
          "::ad::map::match::ObjectReferencePoints::RearRight"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::ObjectReferencePoints::RearRight;
  }
  if (str == std::string("RearRight"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::ObjectReferencePoints::RearRight;
  }
  if (str ==
      std::string(
          "::ad::map::match::ObjectReferencePoints::Center"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::ObjectReferencePoints::Center;
  }
  if (str == std::string("Center"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::ObjectReferencePoints::Center;
  }
  if (str ==
      std::string(
          "::ad::map::match::ObjectReferencePoints::NumPoints"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::ObjectReferencePoints::NumPoints;
  }
  if (str == std::string("NumPoints"))  // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::match::ObjectReferencePoints::NumPoints;
  }
  throw std::out_of_range("Invalid enum literal");  // LCOV_EXCL_BR_LINE
}
