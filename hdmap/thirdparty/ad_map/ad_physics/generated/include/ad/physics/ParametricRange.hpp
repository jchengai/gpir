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
#include "ad/physics/ParametricValue.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace physics
 */
namespace physics {

/*!
 * \brief DataType ParametricRange
 *
 * A parametric range within a lane segment described by its borders: [minimum, maximum].
 */
struct ParametricRange
{
  /*!
   * \brief Smart pointer on ParametricRange
   */
  typedef std::shared_ptr<ParametricRange> Ptr;

  /*!
   * \brief Smart pointer on constant ParametricRange
   */
  typedef std::shared_ptr<ParametricRange const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  ParametricRange() = default;

  /*!
   * \brief standard destructor
   */
  ~ParametricRange() = default;

  /*!
   * \brief standard copy constructor
   */
  ParametricRange(const ParametricRange &other) = default;

  /*!
   * \brief standard move constructor
   */
  ParametricRange(ParametricRange &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ParametricRange
   *
   * \returns Reference to this ParametricRange.
   */
  ParametricRange &operator=(const ParametricRange &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ParametricRange
   *
   * \returns Reference to this ParametricRange.
   */
  ParametricRange &operator=(ParametricRange &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ParametricRange
   *
   * \returns \c true if both ParametricRange are equal
   */
  bool operator==(const ParametricRange &other) const
  {
    return (minimum == other.minimum) && (maximum == other.maximum);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ParametricRange.
   *
   * \returns \c true if both ParametricRange are different
   */
  bool operator!=(const ParametricRange &other) const
  {
    return !operator==(other);
  }

  /*!
   * The minimum value of the parametric range.
   */
  ::ad::physics::ParametricValue minimum{0.0};

  /*!
   * The maximum value of the parametric range.
   */
  ::ad::physics::ParametricValue maximum{1.0};
};

} // namespace physics
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_PARAMETRICRANGE
#define GEN_GUARD_AD_PHYSICS_PARAMETRICRANGE
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
 * \param[in] _value ParametricRange value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ParametricRange const &_value)
{
  os << "ParametricRange(";
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
 * \brief overload of the std::to_string for ParametricRange
 */
inline std::string to_string(::ad::physics::ParametricRange const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_PARAMETRICRANGE
