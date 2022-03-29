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
#include "ad/physics/ParametricRange.hpp"
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
 * \brief DataType LaneOccupiedRegion
 *
 * The occupied region of a lane
 */
struct LaneOccupiedRegion
{
  /*!
   * \brief Smart pointer on LaneOccupiedRegion
   */
  typedef std::shared_ptr<LaneOccupiedRegion> Ptr;

  /*!
   * \brief Smart pointer on constant LaneOccupiedRegion
   */
  typedef std::shared_ptr<LaneOccupiedRegion const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  LaneOccupiedRegion() = default;

  /*!
   * \brief standard destructor
   */
  ~LaneOccupiedRegion() = default;

  /*!
   * \brief standard copy constructor
   */
  LaneOccupiedRegion(const LaneOccupiedRegion &other) = default;

  /*!
   * \brief standard move constructor
   */
  LaneOccupiedRegion(LaneOccupiedRegion &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other LaneOccupiedRegion
   *
   * \returns Reference to this LaneOccupiedRegion.
   */
  LaneOccupiedRegion &operator=(const LaneOccupiedRegion &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other LaneOccupiedRegion
   *
   * \returns Reference to this LaneOccupiedRegion.
   */
  LaneOccupiedRegion &operator=(LaneOccupiedRegion &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other LaneOccupiedRegion
   *
   * \returns \c true if both LaneOccupiedRegion are equal
   */
  bool operator==(const LaneOccupiedRegion &other) const
  {
    return (laneId == other.laneId) && (longitudinalRange == other.longitudinalRange)
      && (lateralRange == other.lateralRange);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other LaneOccupiedRegion.
   *
   * \returns \c true if both LaneOccupiedRegion are different
   */
  bool operator!=(const LaneOccupiedRegion &other) const
  {
    return !operator==(other);
  }

  ::ad::map::lane::LaneId laneId{0};
  ::ad::physics::ParametricRange longitudinalRange;
  ::ad::physics::ParametricRange lateralRange;
};

} // namespace match
} // namespace map
} // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage within other data types
 */
#ifndef GEN_GUARD_AD_MAP_MATCH_LANEOCCUPIEDREGION
#define GEN_GUARD_AD_MAP_MATCH_LANEOCCUPIEDREGION
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
 * \param[in] _value LaneOccupiedRegion value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, LaneOccupiedRegion const &_value)
{
  os << "LaneOccupiedRegion(";
  os << "laneId:";
  os << _value.laneId;
  os << ",";
  os << "longitudinalRange:";
  os << _value.longitudinalRange;
  os << ",";
  os << "lateralRange:";
  os << _value.lateralRange;
  os << ")";
  return os;
}

} // namespace match
} // namespace map
} // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for LaneOccupiedRegion
 */
inline std::string to_string(::ad::map::match::LaneOccupiedRegion const &value)
{
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
} // namespace std
#endif // GEN_GUARD_AD_MAP_MATCH_LANEOCCUPIEDREGION
