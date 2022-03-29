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
 * \brief DataType ECEFHeading
 *
 * Normalized directional vector in ECEF coordinate frame defining a heading.
 */
struct ECEFHeading
{
  /*!
   * \brief Smart pointer on ECEFHeading
   */
  typedef std::shared_ptr<ECEFHeading> Ptr;

  /*!
   * \brief Smart pointer on constant ECEFHeading
   */
  typedef std::shared_ptr<ECEFHeading const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  ECEFHeading() = default;

  /*!
   * \brief standard destructor
   */
  ~ECEFHeading() = default;

  /*!
   * \brief standard copy constructor
   */
  ECEFHeading(const ECEFHeading &other) = default;

  /*!
   * \brief standard move constructor
   */
  ECEFHeading(ECEFHeading &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ECEFHeading
   *
   * \returns Reference to this ECEFHeading.
   */
  ECEFHeading &operator=(const ECEFHeading &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ECEFHeading
   *
   * \returns Reference to this ECEFHeading.
   */
  ECEFHeading &operator=(ECEFHeading &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ECEFHeading
   *
   * \returns \c true if both ECEFHeading are equal
   */
  bool operator==(const ECEFHeading &other) const
  {
    return (x == other.x) && (y == other.y) && (z == other.z);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ECEFHeading.
   *
   * \returns \c true if both ECEFHeading are different
   */
  bool operator!=(const ECEFHeading &other) const
  {
    return !operator==(other);
  }

  /*!
   * The x component of the heading
   */
  ::ad::map::point::ECEFCoordinate x{std::numeric_limits<double>::quiet_NaN()};

  /*!
   * The y component of the heading
   */
  ::ad::map::point::ECEFCoordinate y{std::numeric_limits<double>::quiet_NaN()};

  /*!
   * The z component of the heading
   */
  ::ad::map::point::ECEFCoordinate z{std::numeric_limits<double>::quiet_NaN()};
};

} // namespace point
} // namespace map
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_POINT_ECEFHEADING
#define GEN_GUARD_AD_MAP_POINT_ECEFHEADING
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
 * \param[in] _value ECEFHeading value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ECEFHeading const &_value)
{
  os << "ECEFHeading(";
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

} // namespace point
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ECEFHeading
 */
inline std::string to_string(::ad::map::point::ECEFHeading const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_POINT_ECEFHEADING
