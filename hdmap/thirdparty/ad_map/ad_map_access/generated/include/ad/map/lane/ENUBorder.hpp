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

#include "ad/map/point/ENUEdge.hpp"
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
 * \brief DataType ENUBorder
 *
 * Border in local ENU(east, north, up) coordinate system
 */
struct ENUBorder {
  /*!
   * \brief Smart pointer on ENUBorder
   */
  typedef std::shared_ptr<ENUBorder> Ptr;

  /*!
   * \brief Smart pointer on constant ENUBorder
   */
  typedef std::shared_ptr<ENUBorder const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  ENUBorder() = default;

  /*!
   * \brief standard destructor
   */
  ~ENUBorder() = default;

  /*!
   * \brief standard copy constructor
   */
  ENUBorder(const ENUBorder &other) = default;

  /*!
   * \brief standard move constructor
   */
  ENUBorder(ENUBorder &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ENUBorder
   *
   * \returns Reference to this ENUBorder.
   */
  ENUBorder &operator=(const ENUBorder &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ENUBorder
   *
   * \returns Reference to this ENUBorder.
   */
  ENUBorder &operator=(ENUBorder &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUBorder
   *
   * \returns \c true if both ENUBorder are equal
   */
  bool operator==(const ENUBorder &other) const {
    return (left == other.left) && (right == other.right);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ENUBorder.
   *
   * \returns \c true if both ENUBorder are different
   */
  bool operator!=(const ENUBorder &other) const { return !operator==(other); }

  /*!
   * Left edge of border
   */
  ::ad::map::point::ENUEdge left;

  /*!
   * Right edge of border
   */
  ::ad::map::point::ENUEdge right;
};

}  // namespace lane
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_LANE_ENUBORDER
#define GEN_GUARD_AD_MAP_LANE_ENUBORDER
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
 * \param[in] _value ENUBorder value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ENUBorder const &_value) {
  os << "ENUBorder(";
  os << "left:";
  os << _value.left;
  os << ",";
  os << "right:";
  os << _value.right;
  os << ")";
  return os;
}

}  // namespace lane
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ENUBorder
 */
inline std::string to_string(::ad::map::lane::ENUBorder const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_LANE_ENUBORDER
