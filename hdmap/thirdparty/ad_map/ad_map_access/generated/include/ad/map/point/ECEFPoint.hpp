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

#include "ad/map/point/ECEFCoordinate.hpp"
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
 * \brief DataType ECEFPoint
 *
 * A geographical location in ECEF coordinate system
 */
struct ECEFPoint {
  /*!
   * \brief Smart pointer on ECEFPoint
   */
  typedef std::shared_ptr<ECEFPoint> Ptr;

  /*!
   * \brief Smart pointer on constant ECEFPoint
   */
  typedef std::shared_ptr<ECEFPoint const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  ECEFPoint() = default;

  /*!
   * \brief standard destructor
   */
  ~ECEFPoint() = default;

  /*!
   * \brief standard copy constructor
   */
  ECEFPoint(const ECEFPoint &other) = default;

  /*!
   * \brief standard move constructor
   */
  ECEFPoint(ECEFPoint &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ECEFPoint
   *
   * \returns Reference to this ECEFPoint.
   */
  ECEFPoint &operator=(const ECEFPoint &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ECEFPoint
   *
   * \returns Reference to this ECEFPoint.
   */
  ECEFPoint &operator=(ECEFPoint &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ECEFPoint
   *
   * \returns \c true if both ECEFPoint are equal
   */
  bool operator==(const ECEFPoint &other) const {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ECEFPoint.
   *
   * \returns \c true if both ECEFPoint are different
   */
  bool operator!=(const ECEFPoint &other) const { return !operator==(other); }

  /*!
   * The x component of the point
   */
  ::ad::map::point::ECEFCoordinate x{std::numeric_limits<double>::quiet_NaN()};

  /*!
   * The y component of the point
   */
  ::ad::map::point::ECEFCoordinate y{std::numeric_limits<double>::quiet_NaN()};

  /*!
   * The z component of the point
   */
  ::ad::map::point::ECEFCoordinate z{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace point
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_POINT_ECEFPOINT
#define GEN_GUARD_AD_MAP_POINT_ECEFPOINT
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
 * \param[in] _value ECEFPoint value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ECEFPoint const &_value) {
  os << "ECEFPoint(";
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

}  // namespace point
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ECEFPoint
 */
inline std::string to_string(::ad::map::point::ECEFPoint const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_POINT_ECEFPOINT
