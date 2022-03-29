// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/match/Types.hpp"
#include "ad/map/point/Types.hpp"

/* @brief namespace admap */
namespace ad {
/* @brief namespace map */
namespace map {
/* @brief namespace match */
namespace match {

/** @brief get the list of ParaPoints out of the map matched positions */
point::ParaPointList getParaPoints(
    MapMatchedPositionConfidenceList const &inMapMatchedPositions);

inline bool isLaneType(MapMatchedPositionType const &mapMatchedPositionType) {
  return mapMatchedPositionType == MapMatchedPositionType::LANE_IN ||
         mapMatchedPositionType == MapMatchedPositionType::LANE_LEFT ||
         mapMatchedPositionType == MapMatchedPositionType::LANE_RIGHT;
}

/**
 * @brief Calculates the objects ENU angle based on the map matched position
 */
point::ENUHeading getObjectENUHeading(
    const match::MapMatchedObjectBoundingBox &mapMatchedBoundingBox);

/**
 * @brief get the signed distance to a lane within the map matched positions
 *
 * This check searches the mapMatchedPositions for the given \a checkLaneId and
 * returns a signed distance value to the lane.
 *
 * @param[in] checkLaneId the lane id to find
 * @param[in] mapMatchedPositions the map matched positions to check
 *
 * @returns:
 * If not part of the mapMatchedPositions:  distance =
 * std::numeric_limits<physics::Distance>::max() If it's found and map matched
 * type is:
 * - MapMatchedPositionType::LANE_IN: distance = 0.
 * - MapMatchedPositionType::LANE_LEFT: distance < 0
 * - MapMatchedPositionType::LANE_RIGHT: distance > 0
 *
 * @throws std::runtime_error if mapMatchedPositions are inconsistent
 */
physics::Distance signedDistanceToLane(
    lane::LaneId const checkLaneId,
    MapMatchedPositionConfidenceList const &mapMatchedPositions);

/** @brief get the para point of the occupied Region in the middle of the
 * longitudinal extend of it */
inline point::ParaPoint getCenterParaPoint(
    match::LaneOccupiedRegion const &occupiedRegion) {
  point::ParaPoint paraPoint;
  paraPoint.laneId = occupiedRegion.laneId;
  paraPoint.parametricOffset = 0.5 * (occupiedRegion.longitudinalRange.maximum +
                                      occupiedRegion.longitudinalRange.minimum);
  return paraPoint;
}

}  // namespace match
}  // namespace map
}  // namespace ad
