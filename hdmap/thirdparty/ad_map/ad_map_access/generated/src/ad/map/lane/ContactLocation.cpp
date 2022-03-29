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

#include "ad/map/lane/ContactLocation.hpp"
#include <stdexcept>

std::string toString(::ad::map::lane::ContactLocation const e)
{
  switch (e)
  {
    case ::ad::map::lane::ContactLocation::INVALID:
      return std::string("::ad::map::lane::ContactLocation::INVALID"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactLocation::UNKNOWN:
      return std::string("::ad::map::lane::ContactLocation::UNKNOWN"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactLocation::LEFT:
      return std::string("::ad::map::lane::ContactLocation::LEFT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactLocation::RIGHT:
      return std::string("::ad::map::lane::ContactLocation::RIGHT"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactLocation::SUCCESSOR:
      return std::string("::ad::map::lane::ContactLocation::SUCCESSOR"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactLocation::PREDECESSOR:
      return std::string("::ad::map::lane::ContactLocation::PREDECESSOR"); // LCOV_EXCL_BR_LINE
    case ::ad::map::lane::ContactLocation::OVERLAP:
      return std::string("::ad::map::lane::ContactLocation::OVERLAP"); // LCOV_EXCL_BR_LINE
    default:
      return std::string("UNKNOWN ENUM VALUE"); // LCOV_EXCL_BR_LINE
  }
}

template <>::ad::map::lane::ContactLocation fromString(std::string const &str)
{
  if (str == std::string("::ad::map::lane::ContactLocation::INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::INVALID;
  }
  if (str == std::string("INVALID")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::INVALID;
  }
  if (str == std::string("::ad::map::lane::ContactLocation::UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::UNKNOWN;
  }
  if (str == std::string("UNKNOWN")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::UNKNOWN;
  }
  if (str == std::string("::ad::map::lane::ContactLocation::LEFT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::LEFT;
  }
  if (str == std::string("LEFT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::LEFT;
  }
  if (str == std::string("::ad::map::lane::ContactLocation::RIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::RIGHT;
  }
  if (str == std::string("RIGHT")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::RIGHT;
  }
  if (str == std::string("::ad::map::lane::ContactLocation::SUCCESSOR")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::SUCCESSOR;
  }
  if (str == std::string("SUCCESSOR")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::SUCCESSOR;
  }
  if (str == std::string("::ad::map::lane::ContactLocation::PREDECESSOR")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::PREDECESSOR;
  }
  if (str == std::string("PREDECESSOR")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::PREDECESSOR;
  }
  if (str == std::string("::ad::map::lane::ContactLocation::OVERLAP")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::OVERLAP;
  }
  if (str == std::string("OVERLAP")) // LCOV_EXCL_BR_LINE
  {
    return ::ad::map::lane::ContactLocation::OVERLAP;
  }
  throw std::out_of_range("Invalid enum literal"); // LCOV_EXCL_BR_LINE
}
