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

#pragma once

#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include "ad/physics/ParametricRange.hpp"
#include "ad/physics/Speed.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace restriction
 *
 * Handling of traffic restrictions
 */
namespace restriction {

/*!
 * \brief DataType SpeedLimit
 */
struct SpeedLimit
{
  /*!
   * \brief Smart pointer on SpeedLimit
   */
  typedef std::shared_ptr<SpeedLimit> Ptr;

  /*!
   * \brief Smart pointer on constant SpeedLimit
   */
  typedef std::shared_ptr<SpeedLimit const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  SpeedLimit() = default;

  /*!
   * \brief standard destructor
   */
  ~SpeedLimit() = default;

  /*!
   * \brief standard copy constructor
   */
  SpeedLimit(const SpeedLimit &other) = default;

  /*!
   * \brief standard move constructor
   */
  SpeedLimit(SpeedLimit &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other SpeedLimit
   *
   * \returns Reference to this SpeedLimit.
   */
  SpeedLimit &operator=(const SpeedLimit &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other SpeedLimit
   *
   * \returns Reference to this SpeedLimit.
   */
  SpeedLimit &operator=(SpeedLimit &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other SpeedLimit
   *
   * \returns \c true if both SpeedLimit are equal
   */
  bool operator==(const SpeedLimit &other) const
  {
    return (speedLimit == other.speedLimit) && (lanePiece == other.lanePiece);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other SpeedLimit.
   *
   * \returns \c true if both SpeedLimit are different
   */
  bool operator!=(const SpeedLimit &other) const
  {
    return !operator==(other);
  }

  ::ad::physics::Speed speedLimit;
  ::ad::physics::ParametricRange lanePiece;
};

} // namespace restriction
} // namespace map
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_RESTRICTION_SPEEDLIMIT
#define GEN_GUARD_AD_MAP_RESTRICTION_SPEEDLIMIT
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace restriction
 *
 * Handling of traffic restrictions
 */
namespace restriction {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value SpeedLimit value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, SpeedLimit const &_value)
{
  os << "SpeedLimit(";
  os << "speedLimit:";
  os << _value.speedLimit;
  os << ",";
  os << "lanePiece:";
  os << _value.lanePiece;
  os << ")";
  return os;
}

} // namespace restriction
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for SpeedLimit
 */
inline std::string to_string(::ad::map::restriction::SpeedLimit const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_RESTRICTION_SPEEDLIMIT
