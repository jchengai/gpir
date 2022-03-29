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
 * \brief DataType Dimension2D
 */
struct Dimension2D
{
  /*!
   * \brief Smart pointer on Dimension2D
   */
  typedef std::shared_ptr<Dimension2D> Ptr;

  /*!
   * \brief Smart pointer on constant Dimension2D
   */
  typedef std::shared_ptr<Dimension2D const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  Dimension2D() = default;

  /*!
   * \brief standard destructor
   */
  ~Dimension2D() = default;

  /*!
   * \brief standard copy constructor
   */
  Dimension2D(const Dimension2D &other) = default;

  /*!
   * \brief standard move constructor
   */
  Dimension2D(Dimension2D &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Dimension2D
   *
   * \returns Reference to this Dimension2D.
   */
  Dimension2D &operator=(const Dimension2D &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Dimension2D
   *
   * \returns Reference to this Dimension2D.
   */
  Dimension2D &operator=(Dimension2D &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Dimension2D
   *
   * \returns \c true if both Dimension2D are equal
   */
  bool operator==(const Dimension2D &other) const
  {
    return (length == other.length) && (width == other.width);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Dimension2D.
   *
   * \returns \c true if both Dimension2D are different
   */
  bool operator!=(const Dimension2D &other) const
  {
    return !operator==(other);
  }

  /*!
   * The length is measured along the x-axis in meters
   */
  ::ad::physics::Distance length;

  /*!
   * The width is measured along the y-axis  in meters
   */
  ::ad::physics::Distance width;
};

} // namespace physics
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_DIMENSION2D
#define GEN_GUARD_AD_PHYSICS_DIMENSION2D
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
 * \param[in] _value Dimension2D value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Dimension2D const &_value)
{
  os << "Dimension2D(";
  os << "length:";
  os << _value.length;
  os << ",";
  os << "width:";
  os << _value.width;
  os << ")";
  return os;
}

} // namespace physics
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Dimension2D
 */
inline std::string to_string(::ad::physics::Dimension2D const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_PHYSICS_DIMENSION2D
