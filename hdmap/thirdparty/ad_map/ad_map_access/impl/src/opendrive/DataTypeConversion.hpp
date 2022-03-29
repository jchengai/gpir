// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <opendrive/types.hpp>
#include "ad/map/landmark/Types.hpp"
#include "ad/map/lane/Types.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace opendrive */
namespace opendrive {

/**
 * @brief Converts an opendrive id into an ad::map::LaneId
 * @param[in] id opendrive::Id
 */
inline lane::LaneId toLaneId(::opendrive::Id id)
{
  return lane::LaneId(id);
}

/**
 * @brief Converts an OpenDRIVE landmark id
 */
inline landmark::LandmarkId toLandmarkId(int id)
{
  return landmark::LandmarkId(size_t(id));
}

/**
 * @brief Converts an OpenDRIVE signal type and subtype to a Landmark Type
 */
landmark::LandmarkType toLandmarkType(int type);

/**
 * @brief Converts an OpenDRIVE traffic sign type and subtype to a TrafficSignType
 */
landmark::TrafficSignType toTrafficSignType(int type, int subtype);

/**
 * @brief Derives the contact type from the given traffic signal type. Used for Right of way.
 */
lane::ContactType toContactType(int type);

/**
 * @brief Determines the contact location of a landmark based on heursitics.
 * @param[in] signalReference ::opendrive::SignalReference the signal reference info
 * @param[in] isJunction bool indicating whether the signal is present inside a lane that belongs to an intersection
 * @returns \c The resulting contact location
 */
lane::ContactLocation toContactLocation(::opendrive::SignalReference const &signalReference, bool const &isJunction);

/**
 * @brief Converts an OpenDRIVE traffic signal type and subtype into a TrafficLightType
 */
landmark::TrafficLightType toTrafficLightType(int type, int subtype);

/**
 * @brief Converts an opendrive::LaneType into a ad::map::lane::Lane::Type
 */
lane::LaneType toLaneType(::opendrive::LaneType const &laneType);

/**
 * @brief Calculates the driving direction of the lane given its attributes
 */
lane::LaneDirection toLaneDirection(::opendrive::Lane const &lane, bool rightHandTraffic = true);

/**
 * @brief Converts an opendrive::Point type (in geo coordinates) to a point::GeoPoint type
 */
point::GeoPoint toGeo(::opendrive::Point const &geoPoint);

/**
 * @brief Converts an opendrive::Point type (in geo coordinates) to a point::ECEFPoint type
 */
point::ECEFPoint toECEF(::opendrive::Point const &geoPoint);

/**
 * @brief Converts an opendrive::Edge type to a point::Geometry type
 * @param[in] edge Opendrive Edge to be converted
 * @returns The equivalent point::Geometry
 */
point::Geometry toGeometry(std::vector<::opendrive::Point> edgePoints);

} // namespace opendrive
} // namespace map
} // namespace ad
