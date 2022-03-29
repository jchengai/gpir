// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

/**
 * @file
 */

#pragma once

#include "ad/map/landmark/LandmarkIdValidInputRange.hpp"
#include "ad/map/landmark/LandmarkValidInputRange.hpp"
#include "ad/map/landmark/Types.hpp"
#include "ad/map/lane/Types.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace landmark */
namespace landmark {

/**
 * @brief checks if the given Landmark is valid
 *
 * The landmark is valid if it's within valid input range.
 */
inline bool isValid(Landmark const &landmark, bool const logErrors = true) {
  return withinValidInputRange(landmark, logErrors);
}

/**
 * @brief checks if the given LandmarkId is valid
 *
 * The landmarkId is valid if it's within valid input range.
 */
inline bool isValid(LandmarkId const &landmarkId, bool const logErrors = true) {
  return withinValidInputRange(landmarkId, logErrors);
}

/**
 * @brief Method to be called to retrieve Landmark::Ptr from the Store.
 * @param[in] id Landmark identifier.
 * @returns Landmark::Ptr with given identifier.
 *          Throws std::invalid_argument if landmark does not exist in the
 * store.
 */
Landmark::ConstPtr getLandmarkPtr(LandmarkId const &id);

/**
 * @brief Method to be called to retrieve Landmark from the Store.
 * @param[in] id Landmark identifier.
 * @returns Landmark with given identifier.
 *          Throws std::invalid_argument if landmark does not exist in the
 * store.
 */
Landmark const &getLandmark(LandmarkId const &id);

/**
 * @returns the enu heading of the landmark
 */
point::ENUHeading getENUHeading(Landmark const &landmark);

/**
 * @returns all Landmarks in the Store.
 */
landmark::LandmarkIdList getLandmarks();

/**
 * @brief Method to be called to retrieve Landmark from the Store and convert to
 * ENU representation.
 * @param[in] id Landmark identifier.
 * @returns Landmark with given identifier.
 * @throws std::invalid_argument if the landmark id does not exist in the store
 */
ENULandmark getENULandmark(LandmarkId const &id);

/**
 * @brief Method to be called to retrieve all the visible landmarks from the
 * given lane
 * @param[in] laneId the lane the landmarks refer to
 * @returns Landmarks visible from given lane.
 */
LandmarkIdList getVisibleLandmarks(lane::LaneId const &laneId);

/**
 * @brief Method to be called to retrieve all the visible landmarks of type
 * landmarkType from the given lane
 * @param[in] landmarkType Landmark type desired.
 * @param[in] laneId the lane the landmarks refer to
 * @returns Landmarks of landmarkType visible from given lane.
 */
LandmarkIdList getVisibleLandmarks(LandmarkType const &landmarkType,
                                   lane::LaneId const &laneId);

/**
 * @brief Method to be called to search the nearest landmarkid given the geo
 * point
 * @param[in] The geo point given
 * @returns returns search result of Landmarkid from given geo point.
 */
LandmarkId uniqueLandmarkId(point::GeoPoint const &geoPoint);

/**
 * @brief Method to be called to retrieve all the visible traffic lights from
 * the given lane
 * @param[in] laneId the lane the landmarks refer to
 * @returns Traffic light landmarks visible from given lane.
 */
inline LandmarkIdList getVisibleTrafficLights(lane::LaneId const &laneId) {
  return getVisibleLandmarks(landmark::LandmarkType::TRAFFIC_LIGHT, laneId);
}

}  // namespace landmark
}  // namespace map
}  // namespace ad
