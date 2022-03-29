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
#include "ad/map/point/Altitude.hpp"
#include "ad/map/point/Latitude.hpp"
#include "ad/map/point/Longitude.hpp"
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
 * \brief DataType GeoPoint
 *
 * A geographical location in WGS-84
 */
struct GeoPoint
{
  /*!
   * \brief Smart pointer on GeoPoint
   */
  typedef std::shared_ptr<GeoPoint> Ptr;

  /*!
   * \brief Smart pointer on constant GeoPoint
   */
  typedef std::shared_ptr<GeoPoint const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  GeoPoint() = default;

  /*!
   * \brief standard destructor
   */
  ~GeoPoint() = default;

  /*!
   * \brief standard copy constructor
   */
  GeoPoint(const GeoPoint &other) = default;

  /*!
   * \brief standard move constructor
   */
  GeoPoint(GeoPoint &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other GeoPoint
   *
   * \returns Reference to this GeoPoint.
   */
  GeoPoint &operator=(const GeoPoint &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other GeoPoint
   *
   * \returns Reference to this GeoPoint.
   */
  GeoPoint &operator=(GeoPoint &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other GeoPoint
   *
   * \returns \c true if both GeoPoint are equal
   */
  bool operator==(const GeoPoint &other) const
  {
    return (longitude == other.longitude) && (latitude == other.latitude) && (altitude == other.altitude);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other GeoPoint.
   *
   * \returns \c true if both GeoPoint are different
   */
  bool operator!=(const GeoPoint &other) const
  {
    return !operator==(other);
  }

  /*!
   * The longitude of the geo point.
   */
  ::ad::map::point::Longitude longitude{std::numeric_limits<double>::quiet_NaN()};

  /*!
   * The latitude of the geo point.
   */
  ::ad::map::point::Latitude latitude{std::numeric_limits<double>::quiet_NaN()};

  /*!
   * The altitude of the geo point.
   */
  ::ad::map::point::Altitude altitude{std::numeric_limits<double>::quiet_NaN()};
};

} // namespace point
} // namespace map
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_POINT_GEOPOINT
#define GEN_GUARD_AD_MAP_POINT_GEOPOINT
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
 * \param[in] _value GeoPoint value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, GeoPoint const &_value)
{
  os << "GeoPoint(";
  os << "longitude:";
  os << _value.longitude;
  os << ",";
  os << "latitude:";
  os << _value.latitude;
  os << ",";
  os << "altitude:";
  os << _value.altitude;
  os << ")";
  return os;
}

} // namespace point
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for GeoPoint
 */
inline std::string to_string(::ad::map::point::GeoPoint const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_POINT_GEOPOINT
