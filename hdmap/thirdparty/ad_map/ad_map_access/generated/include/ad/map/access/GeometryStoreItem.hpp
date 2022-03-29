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

#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace access
 *
 * Accessing map data
 */
namespace access {

/*!
 * \brief DataType GeometryStoreItem
 */
struct GeometryStoreItem
{
  /*!
   * \brief Smart pointer on GeometryStoreItem
   */
  typedef std::shared_ptr<GeometryStoreItem> Ptr;

  /*!
   * \brief Smart pointer on constant GeometryStoreItem
   */
  typedef std::shared_ptr<GeometryStoreItem const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  GeometryStoreItem() = default;

  /*!
   * \brief standard destructor
   */
  ~GeometryStoreItem() = default;

  /*!
   * \brief standard copy constructor
   */
  GeometryStoreItem(const GeometryStoreItem &other) = default;

  /*!
   * \brief standard move constructor
   */
  GeometryStoreItem(GeometryStoreItem &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other GeometryStoreItem
   *
   * \returns Reference to this GeometryStoreItem.
   */
  GeometryStoreItem &operator=(const GeometryStoreItem &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other GeometryStoreItem
   *
   * \returns Reference to this GeometryStoreItem.
   */
  GeometryStoreItem &operator=(GeometryStoreItem &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other GeometryStoreItem
   *
   * \returns \c true if both GeometryStoreItem are equal
   */
  bool operator==(const GeometryStoreItem &other) const
  {
    return (leftEdgeOffset == other.leftEdgeOffset) && (rightEdgeOffset == other.rightEdgeOffset)
      && (leftEdgePoints == other.leftEdgePoints) && (rightEdgePoints == other.rightEdgePoints);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other GeometryStoreItem.
   *
   * \returns \c true if both GeometryStoreItem are different
   */
  bool operator!=(const GeometryStoreItem &other) const
  {
    return !operator==(other);
  }

  uint32_t leftEdgeOffset{0};
  uint32_t rightEdgeOffset{0};
  uint32_t leftEdgePoints{0};
  uint32_t rightEdgePoints{0};
};

} // namespace access
} // namespace map
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_ACCESS_GEOMETRYSTOREITEM
#define GEN_GUARD_AD_MAP_ACCESS_GEOMETRYSTOREITEM
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace access
 *
 * Accessing map data
 */
namespace access {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value GeometryStoreItem value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, GeometryStoreItem const &_value)
{
  os << "GeometryStoreItem(";
  os << "leftEdgeOffset:";
  os << _value.leftEdgeOffset;
  os << ",";
  os << "rightEdgeOffset:";
  os << _value.rightEdgeOffset;
  os << ",";
  os << "leftEdgePoints:";
  os << _value.leftEdgePoints;
  os << ",";
  os << "rightEdgePoints:";
  os << _value.rightEdgePoints;
  os << ")";
  return os;
}

} // namespace access
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for GeometryStoreItem
 */
inline std::string to_string(::ad::map::access::GeometryStoreItem const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_ACCESS_GEOMETRYSTOREITEM
