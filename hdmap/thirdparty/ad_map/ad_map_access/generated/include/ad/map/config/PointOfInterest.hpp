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
#include <string>
#include "ad/map/point/GeoPoint.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace config
 *
 * Configuration
 */
namespace config {

/*!
 * \brief DataType PointOfInterest
 *
 * A Point of interest
 */
struct PointOfInterest
{
  /*!
   * \brief Smart pointer on PointOfInterest
   */
  typedef std::shared_ptr<PointOfInterest> Ptr;

  /*!
   * \brief Smart pointer on constant PointOfInterest
   */
  typedef std::shared_ptr<PointOfInterest const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  PointOfInterest() = default;

  /*!
   * \brief standard destructor
   */
  ~PointOfInterest() = default;

  /*!
   * \brief standard copy constructor
   */
  PointOfInterest(const PointOfInterest &other) = default;

  /*!
   * \brief standard move constructor
   */
  PointOfInterest(PointOfInterest &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other PointOfInterest
   *
   * \returns Reference to this PointOfInterest.
   */
  PointOfInterest &operator=(const PointOfInterest &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other PointOfInterest
   *
   * \returns Reference to this PointOfInterest.
   */
  PointOfInterest &operator=(PointOfInterest &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other PointOfInterest
   *
   * \returns \c true if both PointOfInterest are equal
   */
  bool operator==(const PointOfInterest &other) const
  {
    return (geoPoint == other.geoPoint) && (name == other.name);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other PointOfInterest.
   *
   * \returns \c true if both PointOfInterest are different
   */
  bool operator!=(const PointOfInterest &other) const
  {
    return !operator==(other);
  }

  /*!
   * The geo position of the point
   */
  ::ad::map::point::GeoPoint geoPoint;

  /*!
   * The name of the point of interest.
   */
  std::string name;
};

} // namespace config
} // namespace map
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_CONFIG_POINTOFINTEREST
#define GEN_GUARD_AD_MAP_CONFIG_POINTOFINTEREST
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace config
 *
 * Configuration
 */
namespace config {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value PointOfInterest value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, PointOfInterest const &_value)
{
  os << "PointOfInterest(";
  os << "geoPoint:";
  os << _value.geoPoint;
  os << ",";
  os << "name:";
  os << _value.name;
  os << ")";
  return os;
}

} // namespace config
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for PointOfInterest
 */
inline std::string to_string(::ad::map::config::PointOfInterest const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_CONFIG_POINTOFINTEREST
