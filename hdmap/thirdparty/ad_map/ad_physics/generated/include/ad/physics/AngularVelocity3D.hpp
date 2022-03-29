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
#include "ad/physics/AngularVelocity.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace physics
 */
namespace physics {

/*!
 * \brief DataType AngularVelocity3D
 *
 * The rate of change of an object's angular displacement in three-dimensional space with respect to time.
 */
struct AngularVelocity3D
{
  /*!
   * \brief Smart pointer on AngularVelocity3D
   */
  typedef std::shared_ptr<AngularVelocity3D> Ptr;

  /*!
   * \brief Smart pointer on constant AngularVelocity3D
   */
  typedef std::shared_ptr<AngularVelocity3D const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  AngularVelocity3D() = default;

  /*!
   * \brief standard destructor
   */
  ~AngularVelocity3D() = default;

  /*!
   * \brief standard copy constructor
   */
  AngularVelocity3D(const AngularVelocity3D &other) = default;

  /*!
   * \brief standard move constructor
   */
  AngularVelocity3D(AngularVelocity3D &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other AngularVelocity3D
   *
   * \returns Reference to this AngularVelocity3D.
   */
  AngularVelocity3D &operator=(const AngularVelocity3D &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other AngularVelocity3D
   *
   * \returns Reference to this AngularVelocity3D.
   */
  AngularVelocity3D &operator=(AngularVelocity3D &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularVelocity3D
   *
   * \returns \c true if both AngularVelocity3D are equal
   */
  bool operator==(const AngularVelocity3D &other) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other AngularVelocity3D.
   *
   * \returns \c true if both AngularVelocity3D are different
   */
  bool operator!=(const AngularVelocity3D &other) const
  {
    return !operator==(other);
  }

  /*!
   * The x component of the angular velocity
   */
  ::ad::physics::AngularVelocity x;

  /*!
   * The y component of the angular velocity
   */
  ::ad::physics::AngularVelocity y;

  /*!
   * The z component of the angular velocity
   */
  ::ad::physics::AngularVelocity z;
};

} // namespace physics
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_ANGULARVELOCITY3D
#define GEN_GUARD_AD_PHYSICS_ANGULARVELOCITY3D
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
 * \param[in] _value AngularVelocity3D value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, AngularVelocity3D const &_value)
{
  os << "AngularVelocity3D(";
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
 * \brief overload of the std::to_string for AngularVelocity3D
 */
inline std::string to_string(::ad::physics::AngularVelocity3D const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_ANGULARVELOCITY3D
