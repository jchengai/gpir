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
#include <memory>
#include <sstream>
#include "ad/map/point/GeoEdge.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace lane
 *
 * Handling of lanes
 */
namespace lane {

/*!
 * \brief DataType GeoBorder
 *
 * Lane borders in geo coordinates
 */
struct GeoBorder
{
  /*!
   * \brief Smart pointer on GeoBorder
   */
  typedef std::shared_ptr<GeoBorder> Ptr;

  /*!
   * \brief Smart pointer on constant GeoBorder
   */
  typedef std::shared_ptr<GeoBorder const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  GeoBorder() = default;

  /*!
   * \brief standard destructor
   */
  ~GeoBorder() = default;

  /*!
   * \brief standard copy constructor
   */
  GeoBorder(const GeoBorder &other) = default;

  /*!
   * \brief standard move constructor
   */
  GeoBorder(GeoBorder &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other GeoBorder
   *
   * \returns Reference to this GeoBorder.
   */
  GeoBorder &operator=(const GeoBorder &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other GeoBorder
   *
   * \returns Reference to this GeoBorder.
   */
  GeoBorder &operator=(GeoBorder &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other GeoBorder
   *
   * \returns \c true if both GeoBorder are equal
   */
  bool operator==(const GeoBorder &other) const
  {
    return (left == other.left) && (right == other.right);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other GeoBorder.
   *
   * \returns \c true if both GeoBorder are different
   */
  bool operator!=(const GeoBorder &other) const
  {
    return !operator==(other);
  }

  /*!
   * Left edge of border
   */
  ::ad::map::point::GeoEdge left;

  /*!
   * Right edge of border
   */
  ::ad::map::point::GeoEdge right;
};

} // namespace lane
} // namespace map
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_LANE_GEOBORDER
#define GEN_GUARD_AD_MAP_LANE_GEOBORDER
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace lane
 *
 * Handling of lanes
 */
namespace lane {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value GeoBorder value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, GeoBorder const &_value)
{
  os << "GeoBorder(";
  os << "left:";
  os << _value.left;
  os << ",";
  os << "right:";
  os << _value.right;
  os << ")";
  return os;
}

} // namespace lane
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for GeoBorder
 */
inline std::string to_string(::ad::map::lane::GeoBorder const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_LANE_GEOBORDER
