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
#include "ad/physics/Acceleration.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace physics
 */
namespace physics {

/*!
 * \brief DataType AccelerationRange
 *
 * An acceleration range described by its borders: [minimum, maximum].
 */
struct AccelerationRange
{
  /*!
   * \brief Smart pointer on AccelerationRange
   */
  typedef std::shared_ptr<AccelerationRange> Ptr;

  /*!
   * \brief Smart pointer on constant AccelerationRange
   */
  typedef std::shared_ptr<AccelerationRange const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  AccelerationRange() = default;

  /*!
   * \brief standard destructor
   */
  ~AccelerationRange() = default;

  /*!
   * \brief standard copy constructor
   */
  AccelerationRange(const AccelerationRange &other) = default;

  /*!
   * \brief standard move constructor
   */
  AccelerationRange(AccelerationRange &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other AccelerationRange
   *
   * \returns Reference to this AccelerationRange.
   */
  AccelerationRange &operator=(const AccelerationRange &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other AccelerationRange
   *
   * \returns Reference to this AccelerationRange.
   */
  AccelerationRange &operator=(AccelerationRange &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AccelerationRange
   *
   * \returns \c true if both AccelerationRange are equal
   */
  bool operator==(const AccelerationRange &other) const
  {
    return (minimum == other.minimum) && (maximum == other.maximum);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AccelerationRange.
   *
   * \returns \c true if both AccelerationRange are different
   */
  bool operator!=(const AccelerationRange &other) const
  {
    return !operator==(other);
  }

  /*!
   * The minimum value of the acceleration range.
   */
  ::ad::physics::Acceleration minimum{std::numeric_limits<Acceleration>::lowest()};

  /*!
   * The maximum value of the acceleration range.
   */
  ::ad::physics::Acceleration maximum{std::numeric_limits<Acceleration>::max()};
};

} // namespace physics
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_ACCELERATIONRANGE
#define GEN_GUARD_AD_PHYSICS_ACCELERATIONRANGE
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
 * \param[in] _value AccelerationRange value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, AccelerationRange const &_value)
{
  os << "AccelerationRange(";
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
 * \brief overload of the std::to_string for AccelerationRange
 */
inline std::string to_string(::ad::physics::AccelerationRange const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_ACCELERATIONRANGE
