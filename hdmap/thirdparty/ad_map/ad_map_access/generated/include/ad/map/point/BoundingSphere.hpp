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

#include "ad/map/point/ECEFPoint.hpp"
#include "ad/physics/Distance.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace point
 *
 * Handling geographic positions in different coordinate systems
 */
namespace point {

/*!
 * \brief DataType BoundingSphere
 *
 * Definition of a bounding sphere
 */
struct BoundingSphere {
  /*!
   * \brief Smart pointer on BoundingSphere
   */
  typedef std::shared_ptr<BoundingSphere> Ptr;

  /*!
   * \brief Smart pointer on constant BoundingSphere
   */
  typedef std::shared_ptr<BoundingSphere const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  BoundingSphere() = default;

  /*!
   * \brief standard destructor
   */
  ~BoundingSphere() = default;

  /*!
   * \brief standard copy constructor
   */
  BoundingSphere(const BoundingSphere &other) = default;

  /*!
   * \brief standard move constructor
   */
  BoundingSphere(BoundingSphere &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other BoundingSphere
   *
   * \returns Reference to this BoundingSphere.
   */
  BoundingSphere &operator=(const BoundingSphere &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other BoundingSphere
   *
   * \returns Reference to this BoundingSphere.
   */
  BoundingSphere &operator=(BoundingSphere &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other BoundingSphere
   *
   * \returns \c true if both BoundingSphere are equal
   */
  bool operator==(const BoundingSphere &other) const {
    return (center == other.center) && (radius == other.radius);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other BoundingSphere.
   *
   * \returns \c true if both BoundingSphere are different
   */
  bool operator!=(const BoundingSphere &other) const {
    return !operator==(other);
  }

  /*!
   * Center of the bounding sphere.
   */
  ::ad::map::point::ECEFPoint center;

  /*!
   * Radius of the bounding sphere.
   */
  ::ad::physics::Distance radius;
};

}  // namespace point
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_POINT_BOUNDINGSPHERE
#define GEN_GUARD_AD_MAP_POINT_BOUNDINGSPHERE
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace point
 *
 * Handling geographic positions in different coordinate systems
 */
namespace point {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value BoundingSphere value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os,
                                BoundingSphere const &_value) {
  os << "BoundingSphere(";
  os << "center:";
  os << _value.center;
  os << ",";
  os << "radius:";
  os << _value.radius;
  os << ")";
  return os;
}

}  // namespace point
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for BoundingSphere
 */
inline std::string to_string(::ad::map::point::BoundingSphere const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_POINT_BOUNDINGSPHERE
