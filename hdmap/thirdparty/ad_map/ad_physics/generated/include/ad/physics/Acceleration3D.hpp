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
 * \brief DataType Acceleration3D
 *
 * The rate of change of Velocity of an object with respect to time.
 */
struct Acceleration3D
{
  /*!
   * \brief Smart pointer on Acceleration3D
   */
  typedef std::shared_ptr<Acceleration3D> Ptr;

  /*!
   * \brief Smart pointer on constant Acceleration3D
   */
  typedef std::shared_ptr<Acceleration3D const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  Acceleration3D() = default;

  /*!
   * \brief standard destructor
   */
  ~Acceleration3D() = default;

  /*!
   * \brief standard copy constructor
   */
  Acceleration3D(const Acceleration3D &other) = default;

  /*!
   * \brief standard move constructor
   */
  Acceleration3D(Acceleration3D &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Acceleration3D
   *
   * \returns Reference to this Acceleration3D.
   */
  Acceleration3D &operator=(const Acceleration3D &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Acceleration3D
   *
   * \returns Reference to this Acceleration3D.
   */
  Acceleration3D &operator=(Acceleration3D &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Acceleration3D
   *
   * \returns \c true if both Acceleration3D are equal
   */
  bool operator==(const Acceleration3D &other) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Acceleration3D.
   *
   * \returns \c true if both Acceleration3D are different
   */
  bool operator!=(const Acceleration3D &other) const
  {
    return !operator==(other);
  }

  /*!
   * The x component of the acceleration
   */
  ::ad::physics::Acceleration x;

  /*!
   * The y component of the acceleration
   */
  ::ad::physics::Acceleration y;

  /*!
   * The y component of the acceleration
   */
  ::ad::physics::Acceleration z;
};

} // namespace physics
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_ACCELERATION3D
#define GEN_GUARD_AD_PHYSICS_ACCELERATION3D
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
 * \param[in] _value Acceleration3D value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Acceleration3D const &_value)
{
  os << "Acceleration3D(";
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
 * \brief overload of the std::to_string for Acceleration3D
 */
inline std::string to_string(::ad::physics::Acceleration3D const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_ACCELERATION3D
