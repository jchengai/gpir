// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/point/ParaPoint.hpp"

namespace ad {
namespace map {
namespace point {

/**
 * @brief create a ParaPoint
 *
 * @param[in] laneId  the lane id
 * @param[in] parametricOffset the parametric offset
 */
inline ParaPoint createParaPoint(
    lane::LaneId const &laneId,
    physics::ParametricValue const &parametricOffset) {
  ParaPoint result;
  result.laneId = laneId;
  result.parametricOffset = parametricOffset;
  return result;
}

}  // namespace point
}  // namespace map
}  // namespace ad

/**
 * @brief Standard comparison operator.
 * @returns True if left object can be seen as less than right object.
 */
inline bool operator<(const ::ad::map::point::ParaPoint &left,
                      const ::ad::map::point::ParaPoint &right) {
  if (left.laneId < right.laneId) {
    return true;
  } else if (left.laneId == right.laneId) {
    return left.parametricOffset < right.parametricOffset;
  } else {
    return false;
  }
}

/**
 * @brief Standard comparison operator
 * @returns True if left object can be seen as greater than right object.
 */
inline bool operator>(const ::ad::map::point::ParaPoint &left,
                      const ::ad::map::point::ParaPoint &right) {
  if (left.laneId > right.laneId) {
    return true;
  } else if (left.laneId == right.laneId) {
    return left.parametricOffset > right.parametricOffset;
  } else {
    return false;
  }
}

/**
 * @brief Standard comparison operator.
 * @returns True if left object can be seen as less than or equal to right
 * object.
 */
inline bool operator<=(const ::ad::map::point::ParaPoint &left,
                       const ::ad::map::point::ParaPoint &right) {
  return operator<(left, right) || (left == right);
}

/**
 * @brief Standard comparison operator.
 * @returns True if left object can be seen as greater than or equal to right
 * object.
 */
inline bool operator>=(const ::ad::map::point::ParaPoint &left,
                       const ::ad::map::point::ParaPoint &right) {
  return operator>(left, right) || (left == right);
}
