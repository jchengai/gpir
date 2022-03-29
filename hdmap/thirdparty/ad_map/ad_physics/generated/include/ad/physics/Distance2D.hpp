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
#include "ad/physics/Distance.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace physics
 */
namespace physics {

/*!
 * \brief DataType Distance2D
 *
 * The vector of distances between two points along the axes of a coordinate frame of reference (minimum distance).
 */
struct Distance2D
{
  /*!
   * \brief Smart pointer on Distance2D
   */
  typedef std::shared_ptr<Distance2D> Ptr;

  /*!
   * \brief Smart pointer on constant Distance2D
   */
  typedef std::shared_ptr<Distance2D const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  Distance2D() = default;

  /*!
   * \brief standard destructor
   */
  ~Distance2D() = default;

  /*!
   * \brief standard copy constructor
   */
  Distance2D(const Distance2D &other) = default;

  /*!
   * \brief standard move constructor
   */
  Distance2D(Distance2D &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Distance2D
   *
   * \returns Reference to this Distance2D.
   */
  Distance2D &operator=(const Distance2D &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Distance2D
   *
   * \returns Reference to this Distance2D.
   */
  Distance2D &operator=(Distance2D &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Distance2D
   *
   * \returns \c true if both Distance2D are equal
   */
  bool operator==(const Distance2D &other) const
  {
    return (x == other.x) && (y == other.y);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Distance2D.
   *
   * \returns \c true if both Distance2D are different
   */
  bool operator!=(const Distance2D &other) const
  {
    return !operator==(other);
  }

  ::ad::physics::Distance x;
  ::ad::physics::Distance y;
};

} // namespace physics
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_DISTANCE2D
#define GEN_GUARD_AD_PHYSICS_DISTANCE2D
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
 * \param[in] _value Distance2D value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Distance2D const &_value)
{
  os << "Distance2D(";
  os << "x:";
  os << _value.x;
  os << ",";
  os << "y:";
  os << _value.y;
  os << ")";
  return os;
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Distance2D
 */
inline std::string to_string(::ad::physics::Distance2D const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_DISTANCE2D
