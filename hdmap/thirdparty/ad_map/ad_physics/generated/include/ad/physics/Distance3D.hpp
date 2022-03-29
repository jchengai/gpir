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
 * \brief DataType Distance3D
 *
 * The vector of distances between two points along the axes of a coordinate frame of reference (minimum distance).
 */
struct Distance3D
{
  /*!
   * \brief Smart pointer on Distance3D
   */
  typedef std::shared_ptr<Distance3D> Ptr;

  /*!
   * \brief Smart pointer on constant Distance3D
   */
  typedef std::shared_ptr<Distance3D const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  Distance3D() = default;

  /*!
   * \brief standard destructor
   */
  ~Distance3D() = default;

  /*!
   * \brief standard copy constructor
   */
  Distance3D(const Distance3D &other) = default;

  /*!
   * \brief standard move constructor
   */
  Distance3D(Distance3D &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Distance3D
   *
   * \returns Reference to this Distance3D.
   */
  Distance3D &operator=(const Distance3D &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Distance3D
   *
   * \returns Reference to this Distance3D.
   */
  Distance3D &operator=(Distance3D &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Distance3D
   *
   * \returns \c true if both Distance3D are equal
   */
  bool operator==(const Distance3D &other) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Distance3D.
   *
   * \returns \c true if both Distance3D are different
   */
  bool operator!=(const Distance3D &other) const
  {
    return !operator==(other);
  }

  /*!
   * The x component of the distance
   */
  ::ad::physics::Distance x;

  /*!
   * The y component of the distance
   */
  ::ad::physics::Distance y;

  /*!
   * The z component of the distance
   */
  ::ad::physics::Distance z;
};

} // namespace physics
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_DISTANCE3D
#define GEN_GUARD_AD_PHYSICS_DISTANCE3D
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
 * \param[in] _value Distance3D value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Distance3D const &_value)
{
  os << "Distance3D(";
  os << "x:";
  os << _value.x;
  os << ",";
  os << "y:";
  os << _value.y;
  os << ",";
  os << "z:";
  os << _value.z;
  os << ")";
  return os;
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Distance3D
 */
inline std::string to_string(::ad::physics::Distance3D const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_DISTANCE3D
