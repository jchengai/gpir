// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <set>

#include "ad/map/point/ParaPointOperation.hpp"

/* @brief namespace ad */
namespace ad {
/* @brief namespace map */
namespace map {
/* @brief namespace route */
namespace route {
/**
 * @namespace planning
 * @brief provides route planning capabilities on the road network of the map
 */
namespace planning {

/**
 * @brief direction at the para point in respect to the lane orientation
 */
enum class RoutingDirection {
  DONT_CARE,  ///< Vehicle direction is not relevant.
  POSITIVE,   ///< Vehicle direction is same as Lane orientation.
  NEGATIVE    ///< Vehicle direction is opposite of Lane orientation.
};

/**
 * @brief routing para point
 *
 * It is essential to know in which direction a vehicle is oriented in respect
 * to the lane when trying to route. Therefore, the routing para point extends
 * the para point by a RoutingDirection.
 */
struct RoutingParaPoint {
  /**
   * @brief Standard comparison operator.
   * @returns True if two objects can be taken as equal.
   */
  bool operator==(const RoutingParaPoint &other) const {
    return (direction == other.direction) && (point == other.point);
  }

  /**
   * @brief Standard comparison operator.
   * @returns True if two objects can be taken as distinct.
   */
  bool operator!=(const RoutingParaPoint &other) const {
    return !operator==(other);
  }

  /**
   * @brief Standard comparison operator.
   * @returns True if this objects can be taken as smaller than the other.
   */
  bool operator<(const RoutingParaPoint &other) const {
    if (direction == other.direction) {
      return point < other.point;
    }
    return direction < other.direction;
  }

  point::ParaPoint point;
  RoutingDirection direction{RoutingDirection::DONT_CARE};
};

/**
 * @brief create a RoutingParaPoint
 *
 * @param[in] laneId  the lane id
 * @param[in] parametricOffset the parametric offset
 * @param[in] routingDirection the routing direction in respect to the lane
 * orientation Be aware: this might be different from the nominal driving
 * direction!
 */
inline RoutingParaPoint createRoutingParaPoint(
    lane::LaneId const &laneId,
    physics::ParametricValue const &parametricOffset,
    RoutingDirection const &routingDirection = RoutingDirection::DONT_CARE) {
  RoutingParaPoint routingPoint;
  routingPoint.point.laneId = laneId;
  routingPoint.point.parametricOffset = parametricOffset;
  routingPoint.direction = routingDirection;
  return routingPoint;
}

}  // namespace planning
}  // namespace route
}  // namespace map
}  // namespace ad
