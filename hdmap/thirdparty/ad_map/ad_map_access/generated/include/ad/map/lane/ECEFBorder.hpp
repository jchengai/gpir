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
#include "ad/map/point/ECEFEdge.hpp"
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
 * \brief DataType ECEFBorder
 *
 * Border in ECEF(earth centered, earth fixed) coordinate system
 */
struct ECEFBorder
{
  /*!
   * \brief Smart pointer on ECEFBorder
   */
  typedef std::shared_ptr<ECEFBorder> Ptr;

  /*!
   * \brief Smart pointer on constant ECEFBorder
   */
  typedef std::shared_ptr<ECEFBorder const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  ECEFBorder() = default;

  /*!
   * \brief standard destructor
   */
  ~ECEFBorder() = default;

  /*!
   * \brief standard copy constructor
   */
  ECEFBorder(const ECEFBorder &other) = default;

  /*!
   * \brief standard move constructor
   */
  ECEFBorder(ECEFBorder &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ECEFBorder
   *
   * \returns Reference to this ECEFBorder.
   */
  ECEFBorder &operator=(const ECEFBorder &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ECEFBorder
   *
   * \returns Reference to this ECEFBorder.
   */
  ECEFBorder &operator=(ECEFBorder &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ECEFBorder
   *
   * \returns \c true if both ECEFBorder are equal
   */
  bool operator==(const ECEFBorder &other) const
  {
    return (left == other.left) && (right == other.right);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ECEFBorder.
   *
   * \returns \c true if both ECEFBorder are different
   */
  bool operator!=(const ECEFBorder &other) const
  {
    return !operator==(other);
  }

  /*!
   * Left edge of the border
   */
  ::ad::map::point::ECEFEdge left;

  /*!
   * Right edge of the border
   */
  ::ad::map::point::ECEFEdge right;
};

} // namespace lane
} // namespace map
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_LANE_ECEFBORDER
#define GEN_GUARD_AD_MAP_LANE_ECEFBORDER
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
 * \param[in] _value ECEFBorder value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ECEFBorder const &_value)
{
  os << "ECEFBorder(";
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
 * \brief overload of the std::to_string for ECEFBorder
 */
inline std::string to_string(::ad::map::lane::ECEFBorder const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_LANE_ECEFBORDER
