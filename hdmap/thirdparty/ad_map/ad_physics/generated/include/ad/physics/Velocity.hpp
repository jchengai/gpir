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
 * \brief DataType Velocity
 *
 * The rate of change of an object's position with respect to a frame of reference.
 *
 * Velocity is equivalent to a specification of its speed and direction of motion.
 */
struct Velocity
{
  /*!
   * \brief Smart pointer on Velocity
   */
  typedef std::shared_ptr<Velocity> Ptr;

  /*!
   * \brief Smart pointer on constant Velocity
   */
  typedef std::shared_ptr<Velocity const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  Velocity() = default;

  /*!
   * \brief standard destructor
   */
  ~Velocity() = default;

  /*!
   * \brief standard copy constructor
   */
  Velocity(const Velocity &other) = default;

  /*!
   * \brief standard move constructor
   */
  Velocity(Velocity &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Velocity
   *
   * \returns Reference to this Velocity.
   */
  Velocity &operator=(const Velocity &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Velocity
   *
   * \returns Reference to this Velocity.
   */
  Velocity &operator=(Velocity &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Velocity
   *
   * \returns \c true if both Velocity are equal
   */
  bool operator==(const Velocity &other) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Velocity.
   *
   * \returns \c true if both Velocity are different
   */
  bool operator!=(const Velocity &other) const
  {
    return !operator==(other);
  }

  /*!
   * The x component of the velocity
   */
  ::ad::physics::Speed x;

  /*!
   * The y component of the velocity
   */
  ::ad::physics::Speed y;

  /*!
   * The z component of the velocity
   */
  ::ad::physics::Speed z;
};

} // namespace physics
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_VELOCITY
#define GEN_GUARD_AD_PHYSICS_VELOCITY
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
 * \param[in] _value Velocity value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Velocity const &_value)
{
  os << "Velocity(";
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
 * \brief overload of the std::to_string for Velocity
 */
inline std::string to_string(::ad::physics::Velocity const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_VELOCITY
