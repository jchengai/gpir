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

#include "ad/map/point/ENUCoordinate.hpp"
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
 * \brief DataType ENUPoint
 *
 * Describes a position in ENU Coordinate System
 *
 * ENU Coordinate System is a right handed coordinate system.
 * x-axis pointing east
 * y-axis pointing north
 * z-axis pointing up
 *
 */
struct ENUPoint {
  /*!
   * \brief Smart pointer on ENUPoint
   */
  typedef std::shared_ptr<ENUPoint> Ptr;

  /*!
   * \brief Smart pointer on constant ENUPoint
   */
  typedef std::shared_ptr<ENUPoint const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  ENUPoint() = default;

  ENUPoint(const double _x, const double _y, const double _z) {
    x = _x;
    y = _y;
    z = _z;
  }

  /*!
   * \brief standard destructor
   */
  ~ENUPoint() = default;

  /*!
   * \brief standard copy constructor
   */
  ENUPoint(const ENUPoint &other) = default;

  /*!
   * \brief standard move constructor
   */
  ENUPoint(ENUPoint &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ENUPoint
   *
   * \returns Reference to this ENUPoint.
   */
  ENUPoint &operator=(const ENUPoint &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ENUPoint
   *
   * \returns Reference to this ENUPoint.
   */
  ENUPoint &operator=(ENUPoint &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUPoint
   *
   * \returns \c true if both ENUPoint are equal
   */
  bool operator==(const ENUPoint &other) const {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUPoint.
   *
   * \returns \c true if both ENUPoint are different
   */
  bool operator!=(const ENUPoint &other) const { return !operator==(other); }

  /*!
   * x position in an ENU Coordinate System [m]
   */
  ::ad::map::point::ENUCoordinate x{std::numeric_limits<double>::quiet_NaN()};

  /*!
   * y position in an ENU Coordinate System [m]
   */
  ::ad::map::point::ENUCoordinate y{std::numeric_limits<double>::quiet_NaN()};

  /*!
   * z position in an ENU Coordinate System [m]
   */
  ::ad::map::point::ENUCoordinate z{std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace point
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_POINT_ENUPOINT
#define GEN_GUARD_AD_MAP_POINT_ENUPOINT
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
 * \param[in] _value ENUPoint value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ENUPoint const &_value) {
  os << "ENUPoint(";
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
 * \brief overload of the std::to_string for ENUPoint
 */
inline std::string to_string(::ad::map::point::ENUPoint const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_POINT_ENUPOINT
