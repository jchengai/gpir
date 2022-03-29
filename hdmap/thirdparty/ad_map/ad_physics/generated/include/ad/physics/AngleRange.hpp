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
#include "ad/physics/Angle.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace physics
 */
namespace physics {

/*!
 * \brief DataType AngleRange
 *
 * A range of angles described by its borders: [minimum, maximum].
 */
struct AngleRange
{
  /*!
   * \brief Smart pointer on AngleRange
   */
  typedef std::shared_ptr<AngleRange> Ptr;

  /*!
   * \brief Smart pointer on constant AngleRange
   */
  typedef std::shared_ptr<AngleRange const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  AngleRange() = default;

  /*!
   * \brief standard destructor
   */
  ~AngleRange() = default;

  /*!
   * \brief standard copy constructor
   */
  AngleRange(const AngleRange &other) = default;

  /*!
   * \brief standard move constructor
   */
  AngleRange(AngleRange &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other AngleRange
   *
   * \returns Reference to this AngleRange.
   */
  AngleRange &operator=(const AngleRange &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other AngleRange
   *
   * \returns Reference to this AngleRange.
   */
  AngleRange &operator=(AngleRange &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngleRange
   *
   * \returns \c true if both AngleRange are equal
   */
  bool operator==(const AngleRange &other) const
  {
    return (minimum == other.minimum) && (maximum == other.maximum);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngleRange.
   *
   * \returns \c true if both AngleRange are different
   */
  bool operator!=(const AngleRange &other) const
  {
    return !operator==(other);
  }

  /*!
   * The minimum value of the angle range.
   */
  ::ad::physics::Angle minimum{std::numeric_limits<Angle>::lowest()};

  /*!
   * The maximum value of the angle range.
   */
  ::ad::physics::Angle maximum{std::numeric_limits<Angle>::max()};
};

} // namespace physics
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_ANGLERANGE
#define GEN_GUARD_AD_PHYSICS_ANGLERANGE
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
 * \param[in] _value AngleRange value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, AngleRange const &_value)
{
  os << "AngleRange(";
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
 * \brief overload of the std::to_string for AngleRange
 */
inline std::string to_string(::ad::physics::AngleRange const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_ANGLERANGE
