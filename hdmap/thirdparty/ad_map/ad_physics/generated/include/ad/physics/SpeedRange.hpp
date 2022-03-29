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
#include "ad/physics/Speed.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace physics
 */
namespace physics {

/*!
 * \brief DataType SpeedRange
 *
 * A speed range described by its borders: [minimum, maximum].
 */
struct SpeedRange
{
  /*!
   * \brief Smart pointer on SpeedRange
   */
  typedef std::shared_ptr<SpeedRange> Ptr;

  /*!
   * \brief Smart pointer on constant SpeedRange
   */
  typedef std::shared_ptr<SpeedRange const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  SpeedRange() = default;

  /*!
   * \brief standard destructor
   */
  ~SpeedRange() = default;

  /*!
   * \brief standard copy constructor
   */
  SpeedRange(const SpeedRange &other) = default;

  /*!
   * \brief standard move constructor
   */
  SpeedRange(SpeedRange &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other SpeedRange
   *
   * \returns Reference to this SpeedRange.
   */
  SpeedRange &operator=(const SpeedRange &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other SpeedRange
   *
   * \returns Reference to this SpeedRange.
   */
  SpeedRange &operator=(SpeedRange &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other SpeedRange
   *
   * \returns \c true if both SpeedRange are equal
   */
  bool operator==(const SpeedRange &other) const
  {
    return (minimum == other.minimum) && (maximum == other.maximum);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other SpeedRange.
   *
   * \returns \c true if both SpeedRange are different
   */
  bool operator!=(const SpeedRange &other) const
  {
    return !operator==(other);
  }

  /*!
   * The minimum value of the speed range.
   */
  ::ad::physics::Speed minimum{std::numeric_limits<Speed>::lowest()};

  /*!
   * The maximum value of the speed range.
   */
  ::ad::physics::Speed maximum{std::numeric_limits<Speed>::max()};
};

} // namespace physics
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_SPEEDRANGE
#define GEN_GUARD_AD_PHYSICS_SPEEDRANGE
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace physics
 */
namespace physics {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value SpeedRange value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, SpeedRange const &_value)
{
  os << "SpeedRange(";
  os << "minimum:";
  os << _value.minimum;
  os << ",";
  os << "maximum:";
  os << _value.maximum;
  os << ")";
  return os;
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for SpeedRange
 */
inline std::string to_string(::ad::physics::SpeedRange const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_SPEEDRANGE
