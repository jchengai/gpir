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
 * \brief DataType Dimension3D
 */
struct Dimension3D {
  /*!
   * \brief Smart pointer on Dimension3D
   */
  typedef std::shared_ptr<Dimension3D> Ptr;

  /*!
   * \brief Smart pointer on constant Dimension3D
   */
  typedef std::shared_ptr<Dimension3D const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  Dimension3D() = default;

  /*!
   * \brief standard destructor
   */
  ~Dimension3D() = default;

  /*!
   * \brief standard copy constructor
   */
  Dimension3D(const Dimension3D &other) = default;

  /*!
   * \brief standard move constructor
   */
  Dimension3D(Dimension3D &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Dimension3D
   *
   * \returns Reference to this Dimension3D.
   */
  Dimension3D &operator=(const Dimension3D &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Dimension3D
   *
   * \returns Reference to this Dimension3D.
   */
  Dimension3D &operator=(Dimension3D &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Dimension3D
   *
   * \returns \c true if both Dimension3D are equal
   */
  bool operator==(const Dimension3D &other) const {
    return (length == other.length) && (width == other.width) &&
           (height == other.height);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Dimension3D.
   *
   * \returns \c true if both Dimension3D are different
   */
  bool operator!=(const Dimension3D &other) const { return !operator==(other); }

  /*!
   * The length is measured along the x-axis in meters
   */
  ::ad::physics::Distance length;

  /*!
   * The width is measured along the y-axis  in meters
   */
  ::ad::physics::Distance width;

  /*!
   * The height is measured along the z-axis  in meters
   */
  ::ad::physics::Distance height;
};

}  // namespace physics
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_PHYSICS_DIMENSION3D
#define GEN_GUARD_AD_PHYSICS_DIMENSION3D
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
 * \param[in] _value Dimension3D value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Dimension3D const &_value) {
  os << "Dimension3D(";
  os << "length:";
  os << _value.length;
  os << ",";
  os << "width:";
  os << _value.width;
  os << ",";
  os << "height:";
  os << _value.height;
  os << ")";
  return os;
}

}  // namespace physics
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Dimension3D
 */
inline std::string to_string(::ad::physics::Dimension3D const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_PHYSICS_DIMENSION3D
