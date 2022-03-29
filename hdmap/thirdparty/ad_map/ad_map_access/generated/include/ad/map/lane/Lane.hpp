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

#include "ad/map/landmark/LandmarkIdList.hpp"
#include "ad/map/lane/ComplianceVersion.hpp"
#include "ad/map/lane/ContactLaneList.hpp"
#include "ad/map/lane/LaneDirection.hpp"
#include "ad/map/lane/LaneId.hpp"
#include "ad/map/lane/LaneType.hpp"
#include "ad/map/point/BoundingSphere.hpp"
#include "ad/map/point/Geometry.hpp"
#include "ad/map/restriction/Restrictions.hpp"
#include "ad/map/restriction/SpeedLimitList.hpp"
#include "ad/physics/Distance.hpp"
#include "ad/physics/MetricRange.hpp"
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
 * \brief DataType Lane
 *
 * Specification of a lane
 */
struct Lane {
  /*!
   * \brief Smart pointer on Lane
   */
  typedef std::shared_ptr<Lane> Ptr;

  /*!
   * \brief Smart pointer on constant Lane
   */
  typedef std::shared_ptr<Lane const> ConstPtr;

  /*!
   * \brief standard constructor
   */
  Lane() = default;

  /*!
   * \brief standard destructor
   */
  ~Lane() = default;

  /*!
   * \brief standard copy constructor
   */
  Lane(const Lane &other) = default;

  /*!
   * \brief standard move constructor
   */
  Lane(Lane &&other) = default;

  /**
   * \brief standard assignment operator
   *
   * \param[in] other Other Lane
   *
   * \returns Reference to this Lane.
   */
  Lane &operator=(const Lane &other) = default;

  /**
   * \brief standard move operator
   *
   * \param[in] other Other Lane
   *
   * \returns Reference to this Lane.
   */
  Lane &operator=(Lane &&other) = default;

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Lane
   *
   * \returns \c true if both Lane are equal
   */
  bool operator==(const Lane &other) const {
    return (id == other.id) && (type == other.type) &&
           (direction == other.direction) &&
           (restrictions == other.restrictions) && (length == other.length) &&
           (lengthRange == other.lengthRange) && (width == other.width) &&
           (widthRange == other.widthRange) &&
           (speedLimits == other.speedLimits) && (edgeLeft == other.edgeLeft) &&
           (edgeRight == other.edgeRight) &&
           (contactLanes == other.contactLanes) &&
           (complianceVersion == other.complianceVersion) &&
           (boundingSphere == other.boundingSphere) &&
           (visibleLandmarks == other.visibleLandmarks);
  }

  /**
   * \brief standard comparison operator
   *
   * \param[in] other Other Lane.
   *
   * \returns \c true if both Lane are different
   */
  bool operator!=(const Lane &other) const { return !operator==(other); }

  /*!
   * Identifier of the lane
   */
  ::ad::map::lane::LaneId id;

  /*!
   * Type of the lane
   */
  ::ad::map::lane::LaneType type;

  /*!
   * Driving direction of the lane
   */
  ::ad::map::lane::LaneDirection direction{
      ::ad::map::lane::LaneDirection::INVALID};

  /*!
   * List of restrictions
   */
  ::ad::map::restriction::Restrictions restrictions;

  /*!
   * Length of the lane
   */
  ::ad::physics::Distance length;
  ::ad::physics::MetricRange lengthRange;

  /*!
   * Width of the lane
   */
  ::ad::physics::Distance width;
  ::ad::physics::MetricRange widthRange;

  /*!
   * List of speed limits
   */
  ::ad::map::restriction::SpeedLimitList speedLimits;

  /*!
   * Left edge of the lane
   */
  ::ad::map::point::Geometry edgeLeft;

  /*!
   * Right edge of the lane
   */
  ::ad::map::point::Geometry edgeRight;

  // ! center line of the lane [not included in original ad_map lib]
  mutable ::ad::map::point::ENUEdge center;

  /*!
   * List of contacting lanes
   */
  ::ad::map::lane::ContactLaneList contactLanes;
  ::ad::map::lane::ComplianceVersion complianceVersion;
  ::ad::map::point::BoundingSphere boundingSphere;

  /*!
   * List of visible landmarks
   */
  ::ad::map::landmark::LandmarkIdList visibleLandmarks;
};

}  // namespace lane
}  // namespace map
}  // namespace ad

/*!
 * \brief protect the definition of functions from duplicates by typedef usage
 * within other data types
 */
#ifndef GEN_GUARD_AD_MAP_LANE_LANE
#define GEN_GUARD_AD_MAP_LANE_LANE
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
 * \param[in] _value Lane value
 *
 * \returns The stream object.
 *
 */
inline std::ostream &operator<<(std::ostream &os, Lane const &_value) {
  os << "Lane(";
  os << "id:";
  os << _value.id;
  os << ",";
  os << "type:";
  os << _value.type;
  os << ",";
  os << "direction:";
  os << _value.direction;
  os << ",";
  os << "restrictions:";
  os << _value.restrictions;
  os << ",";
  os << "length:";
  os << _value.length;
  os << ",";
  os << "lengthRange:";
  os << _value.lengthRange;
  os << ",";
  os << "width:";
  os << _value.width;
  os << ",";
  os << "widthRange:";
  os << _value.widthRange;
  os << ",";
  os << "speedLimits:";
  os << _value.speedLimits;
  os << ",";
  os << "edgeLeft:";
  os << _value.edgeLeft;
  os << ",";
  os << "edgeRight:";
  os << _value.edgeRight;
  os << ",";
  os << "contactLanes:";
  os << _value.contactLanes;
  os << ",";
  os << "complianceVersion:";
  os << _value.complianceVersion;
  os << ",";
  os << "boundingSphere:";
  os << _value.boundingSphere;
  os << ",";
  os << "visibleLandmarks:";
  os << _value.visibleLandmarks;
  os << ")";
  return os;
}

}  // namespace lane
}  // namespace map
}  // namespace ad

namespace std {
/*!
 * \brief overload of the std::to_string for Lane
 */
inline std::string to_string(::ad::map::lane::Lane const &value) {
  stringstream sstream;
  sstream << value;
  return sstream.str();
}
}  // namespace std
#endif  // GEN_GUARD_AD_MAP_LANE_LANE
