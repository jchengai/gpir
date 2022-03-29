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

#include "ad/map/point/ParaPoint.hpp"
#include "ad/physics/Distance.hpp"
#include "ad/physics/RatioValue.hpp"
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace match
 *
 * Map matching
 */
namespace match {

/*!
 * \brief DataType LanePoint
 *
 * Defines a point on a lane.
 * Besides the parametric offsets in longitudinal and lateral direction also the
 * length and width of the lane at this point.
 */
struct LanePoint {
  /*!
   * \brief Smart pointer on LanePoint
   */
  typedef std::shared_ptr<LanePoint> Ptr;

  /*!
   * \brief Smart pointer on constant LanePoint
   */
  typedef std::shared_ptr<LanePoint const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  LanePoint() = default;

  /*!
   * \brief standard destructor
   */
  ~LanePoint() = default;

  /*!
   * \brief standard copy constructor
   */
  LanePoint(const LanePoint &other) = default;

  /*!
   * \brief standard move constructor
   */
  LanePoint(LanePoint &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other LanePoint
   *
   * \returns Reference to this LanePoint.
   */
  LanePoint &operator=(const LanePoint &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other LanePoint
   *
   * \returns Reference to this LanePoint.
   */
  LanePoint &operator=(LanePoint &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other LanePoint
   *
   * \returns \c true if both LanePoint are equal
   */
  bool operator==(const LanePoint &other) const {
    return (paraPoint == other.paraPoint) && (lateralT == other.lateralT) &&
           (laneLength == other.laneLength) && (laneWidth == other.laneWidth);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other LanePoint.
   *
   * \returns \c true if both LanePoint are different
   */
  bool operator!=(const LanePoint &other) const { return !operator==(other); }

  ::ad::map::point::ParaPoint paraPoint;
  ::ad::physics::RatioValue lateralT;

  /*!
   * Length of lane
   */
  ::ad::physics::Distance laneLength;

  /*!
   * Width of lane
   */
  ::ad::physics::Distance laneWidth;
};

}  // namespace match
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_MATCH_LANEPOINT
#define GEN_GUARD_AD_MAP_MATCH_LANEPOINT
/*!
 * @brief namespace ad
 */
namespace ad {
/*!
 * @brief namespace map
 */
namespace map {
/*!
 * @brief namespace match
 *
 * Map matching
 */
namespace match {

/**
 * \brief standard ostream operator
 *
 * \param[in] os The output stream to write to
 * \param[in] _value LanePoint value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, LanePoint const &_value) {
  os << "LanePoint(";
  os << "paraPoint:";
  os << _value.paraPoint;
  os << ",";
  os << "lateralT:";
  os << _value.lateralT;
  os << ",";
  os << "laneLength:";
  os << _value.laneLength;
  os << ",";
  os << "laneWidth:";
  os << _value.laneWidth;
  os << ")";
  return os;
}

}  // namespace match
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for LanePoint
 */
inline std::string to_string(::ad::map::match::LanePoint const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_MATCH_LANEPOINT
