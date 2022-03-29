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

#include "ad/map/match/LanePoint.hpp"
#include "ad/map/match/MapMatchedPositionType.hpp"
#include "ad/map/point/ECEFPoint.hpp"
#include "ad/physics/Distance.hpp"
#include "ad/physics/Probability.hpp"
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
 * \brief DataType MapMatchedPosition
 *
 * A map matched position
 */
struct MapMatchedPosition {
  /*!
   * \brief Smart pointer on MapMatchedPosition
   */
  typedef std::shared_ptr<MapMatchedPosition> Ptr;

  /*!
   * \brief Smart pointer on constant MapMatchedPosition
   */
  typedef std::shared_ptr<MapMatchedPosition const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  MapMatchedPosition() = default;

  /*!
   * \brief standard destructor
   */
  ~MapMatchedPosition() = default;

  /*!
   * \brief standard copy constructor
   */
  MapMatchedPosition(const MapMatchedPosition &other) = default;

  /*!
   * \brief standard move constructor
   */
  MapMatchedPosition(MapMatchedPosition &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other MapMatchedPosition
   *
   * \returns Reference to this MapMatchedPosition.
   */
  MapMatchedPosition &operator=(const MapMatchedPosition &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other MapMatchedPosition
   *
   * \returns Reference to this MapMatchedPosition.
   */
  MapMatchedPosition &operator=(MapMatchedPosition &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other MapMatchedPosition
   *
   * \returns \c true if both MapMatchedPosition are equal
   */
  bool operator==(const MapMatchedPosition &other) const {
    return (lanePoint == other.lanePoint) && (type == other.type) &&
           (matchedPoint == other.matchedPoint) &&
           (probability == other.probability) &&
           (queryPoint == other.queryPoint) &&
           (matchedPointDistance == other.matchedPointDistance);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other MapMatchedPosition.
   *
   * \returns \c true if both MapMatchedPosition are different
   */
  bool operator!=(const MapMatchedPosition &other) const {
    return !operator==(other);
  }

  /*!
   * matched point as LanePoint.
   * parametricOffset
   */
  ::ad::map::match::LanePoint lanePoint;
  ::ad::map::match::MapMatchedPositionType type{
      ::ad::map::match::MapMatchedPositionType::INVALID};
  ::ad::map::point::ECEFPoint matchedPoint;
  ::ad::physics::Probability probability;

  /*!
   * Point that is used for determining this matched pos.
   */
  ::ad::map::point::ECEFPoint queryPoint;

  /*!
   * The distance between the matchedPoint and the queryPoint.
   */
  ::ad::physics::Distance matchedPointDistance;
};

}  // namespace match
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_MATCH_MAPMATCHEDPOSITION
#define GEN_GUARD_AD_MAP_MATCH_MAPMATCHEDPOSITION
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
 * \param[in] _value MapMatchedPosition value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os,
                                MapMatchedPosition const &_value) {
  os << "MapMatchedPosition(";
  os << "lanePoint:";
  os << _value.lanePoint;
  os << ",";
  os << "type:";
  os << _value.type;
  os << ",";
  os << "matchedPoint:";
  os << _value.matchedPoint;
  os << ",";
  os << "probability:";
  os << _value.probability;
  os << ",";
  os << "queryPoint:";
  os << _value.queryPoint;
  os << ",";
  os << "matchedPointDistance:";
  os << _value.matchedPointDistance;
  os << ")";
  return os;
}

}  // namespace match
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for MapMatchedPosition
 */
inline std::string to_string(
    ::ad::map::match::MapMatchedPosition const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_MATCH_MAPMATCHEDPOSITION
