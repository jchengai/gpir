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

#include "ad/map/lane/LaneId.hpp"
#include "ad/physics/ParametricValue.hpp"
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
 * \brief DataType ParaPoint
 *
 * Defines a parametric point on a lane of the map.
 * The parametric offset refers to the lanes geometrical representation.
 * Be aware: It is independent from the logical nominal driving direction of a
 * lane.
 */
struct ParaPoint {
  /*!
   * \brief Smart pointer on ParaPoint
   */
  typedef std::shared_ptr<ParaPoint> Ptr;

  /*!
   * \brief Smart pointer on constant ParaPoint
   */
  typedef std::shared_ptr<ParaPoint const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  ParaPoint() = default;

  /*!
   * \brief standard destructor
   */
  ~ParaPoint() = default;

  /*!
   * \brief standard copy constructor
   */
  ParaPoint(const ParaPoint &other) = default;

  /*!
   * \brief standard move constructor
   */
  ParaPoint(ParaPoint &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other ParaPoint
   *
   * \returns Reference to this ParaPoint.
   */
  ParaPoint &operator=(const ParaPoint &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other ParaPoint
   *
   * \returns Reference to this ParaPoint.
   */
  ParaPoint &operator=(ParaPoint &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ParaPoint
   *
   * \returns \c true if both ParaPoint are equal
   */
  bool operator==(const ParaPoint &other) const {
    return (laneId == other.laneId) &&
           (parametricOffset == other.parametricOffset);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other ParaPoint.
   *
   * \returns \c true if both ParaPoint are different
   */
  bool operator!=(const ParaPoint &other) const { return !operator==(other); }

  /*!
   * The id of the lane this parametric point belongs to.
   */
  ::ad::map::lane::LaneId laneId{0};

  /*!
   * The parametric offset in the range of [0;1] within the lane's geometry as
   * defined in the map. 0.0 refers to the start of the lanes points. 1.0 refers
   * to the end of the lanes points. 0.5 refers to in between at half length of
   * the lane. Be aware: Depending on the route direction on the lane either the
   * parametric offset 0.0 or 1.0 can define the start point of that route on
   * that lane.
   */
  ::ad::physics::ParametricValue parametricOffset;
};

}  // namespace point
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_POINT_PARAPOINT
#define GEN_GUARD_AD_MAP_POINT_PARAPOINT
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
 * \param[in] _value ParaPoint value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, ParaPoint const &_value) {
  os << "ParaPoint(";
  os << "laneId:";
  os << _value.laneId;
  os << ",";
  os << "parametricOffset:";
  os << _value.parametricOffset;
  os << ")";
  return os;
}

}  // namespace point
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for ParaPoint
 */
inline std::string to_string(::ad::map::point::ParaPoint const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_POINT_PARAPOINT
