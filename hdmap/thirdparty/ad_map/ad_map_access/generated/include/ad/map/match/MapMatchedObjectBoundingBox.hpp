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

#include "ad/map/match/LaneOccupiedRegionList.hpp"
#include "ad/map/match/MapMatchedObjectReferencePositionList.hpp"
#include "ad/physics/Distance.hpp"
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
 * \brief DataType MapMatchedObjectBoundingBox
 */
struct MapMatchedObjectBoundingBox {
  /*!
   * \brief Smart pointer on MapMatchedObjectBoundingBox
   */
  typedef std::shared_ptr<MapMatchedObjectBoundingBox> Ptr;

  /*!
   * \brief Smart pointer on constant MapMatchedObjectBoundingBox
   */
  typedef std::shared_ptr<MapMatchedObjectBoundingBox const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  MapMatchedObjectBoundingBox() = default;

  /*!
   * \brief standard destructor
   */
  ~MapMatchedObjectBoundingBox() = default;

  /*!
   * \brief standard copy constructor
   */
  MapMatchedObjectBoundingBox(const MapMatchedObjectBoundingBox &other) =
      default;

  /*!
   * \brief standard move constructor
   */
  MapMatchedObjectBoundingBox(MapMatchedObjectBoundingBox &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other MapMatchedObjectBoundingBox
   *
   * \returns Reference to this MapMatchedObjectBoundingBox.
   */
  MapMatchedObjectBoundingBox &operator=(
      const MapMatchedObjectBoundingBox &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other MapMatchedObjectBoundingBox
   *
   * \returns Reference to this MapMatchedObjectBoundingBox.
   */
  MapMatchedObjectBoundingBox &operator=(MapMatchedObjectBoundingBox &&other) =
      default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other MapMatchedObjectBoundingBox
   *
   * \returns \c true if both MapMatchedObjectBoundingBox are equal
   */
  bool operator==(const MapMatchedObjectBoundingBox &other) const {
    return (laneOccupiedRegions == other.laneOccupiedRegions) &&
           (referencePointPositions == other.referencePointPositions) &&
           (samplingDistance == other.samplingDistance) &&
           (matchRadius == other.matchRadius);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other MapMatchedObjectBoundingBox.
   *
   * \returns \c true if both MapMatchedObjectBoundingBox are different
   */
  bool operator!=(const MapMatchedObjectBoundingBox &other) const {
    return !operator==(other);
  }

  ::ad::map::match::LaneOccupiedRegionList laneOccupiedRegions;
  ::ad::map::match::MapMatchedObjectReferencePositionList
      referencePointPositions;

  /*!
   * Sampling distance used to calculate the bounding box.
   */
  ::ad::physics::Distance samplingDistance{0.0};

  /*!
   * The actual map matching radius around the object.
   */
  ::ad::physics::Distance matchRadius{0.0};
};

}  // namespace match
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_MATCH_MAPMATCHEDOBJECTBOUNDINGBOX
#define GEN_GUARD_AD_MAP_MATCH_MAPMATCHEDOBJECTBOUNDINGBOX
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
 * \param[in] _value MapMatchedObjectBoundingBox value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os,
                                MapMatchedObjectBoundingBox const &_value) {
  os << "MapMatchedObjectBoundingBox(";
  os << "laneOccupiedRegions:";
  os << _value.laneOccupiedRegions;
  os << ",";
  os << "referencePointPositions:";
  os << _value.referencePointPositions;
  os << ",";
  os << "samplingDistance:";
  os << _value.samplingDistance;
  os << ",";
  os << "matchRadius:";
  os << _value.matchRadius;
  os << ")";
  return os;
}

}  // namespace match
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for MapMatchedObjectBoundingBox
 */
inline std::string to_string(
    ::ad::map::match::MapMatchedObjectBoundingBox const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_MATCH_MAPMATCHEDOBJECTBOUNDINGBOX
