// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/lane/LaneIdValidInputRange.hpp"
#include "ad/map/lane/LaneValidInputRange.hpp"
#include "ad/map/lane/Types.hpp"
#include "ad/map/match/Types.hpp"
#include "ad/map/point/BoundingSphereOperation.hpp"
#include "ad/map/point/Types.hpp"
#include "ad/map/restriction/RestrictionOperation.hpp"
#include "ad/physics/Duration.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace lane */
namespace lane {

/**
 * @brief checks if the given LaneId is valid
 *
 * The laneId is valid if it's within valid input range.
 */
inline bool isValid(LaneId const &laneId, bool const logErrors = true) {
  return withinValidInputRange(laneId, logErrors);
}

/**
 * @brief checks if the given Lane is valid
 *
 * The lane is valid if it's within valid input range.
 */
inline bool isValid(Lane const &lane, bool const logErrors = true) {
  return withinValidInputRange(lane, logErrors);
}

/**
 * @brief Method to be called to retrieve Lane from the Store.
 * @param[in] id Lane identifier.
 * @returns Lane with given identifier.
 *          Throws std::invalid_argument if id is not valid or if lane does not
 * exist in the store.
 */
Lane::ConstPtr getLanePtr(LaneId const &id);

/**
 * @brief Method to be called to retrieve Lane from the Store.
 * @param[in] id Lane identifier.
 * @returns Lane with given identifier.
 *          Throws std::invalid_argument if id is not valid or if lane does not
 * exist in the store.
 */
Lane const &getLane(lane::LaneId const &id);

/**
 * @brief Method to be called to retrieve identifiers of all Lanes in the Store.
 * @returns Identifiers of all lanes in the store.
 */
LaneIdList getLanes();

/**
 * @return lane heading at a mapMatchedPosition
 */
point::ECEFHeading getLaneECEFHeading(
    match::MapMatchedPosition const &mapMatchedPosition);

/**
 * @return lane heading at a paraPoint
 */
point::ECEFHeading getLaneECEFHeading(point::ParaPoint const &paraPoint);

/**
 * @return lane heading at a mapMatchedPosition
 */
point::ENUHeading getLaneENUHeading(
    match::MapMatchedPosition const &mapMatchedPosition);

/**
 * @return lane heading at a paraPoint with given gnssReference
 */
point::ENUHeading getLaneENUHeading(point::ParaPoint const &paraPoint,
                                    point::GeoPoint const &gnssReference);

/**
 * @return lane id at given location
 *
 * Throws if there is more than one lane at the given position
 */
LaneId uniqueLaneId(point::GeoPoint const &point);

/**
 * @return parametric point at given location
 *
 * Throws if there is more than one lane at the given position
 */
point::ParaPoint uniqueParaPoint(point::GeoPoint const &point);

/**
 * @brief Checks if vehicle fits the lanes restriction criteria.
 * @param[in] lane the lane.
 * @param[in] vehicle Description of the vehicle.
 * @returns true of vehicle fits the restrictions criteria.
 */
inline bool isAccessOk(Lane const &lane,
                       restriction::VehicleDescriptor const &vehicle) {
  return isAccessOk(lane.restrictions, vehicle);
}

/**
 * @brief Checks if given sphere intersects with lane bounding ball.
 * @param[in] lane the lane.
 * @param[in] boundingSphere input sphere
 * @returns true given sphere intersects with lane bounding ball.
 */
inline bool isNear(Lane const &lane,
                   point::BoundingSphere const &boundingSphere) {
  return distance(lane.boundingSphere, boundingSphere) == physics::Distance(0.);
}

/**
 * @param[in] lane the lane.
 * @param[in] longitudinalOffset the longitudinal parametric offset
 * @returns Width of the lane at the given longitudinal parametric offset \a
 * longitudinalOffset
 */
physics::Distance getWidth(Lane const &lane,
                           physics::ParametricValue const &longitudinalOffset);

/**
 * @returns Speed limits on the provided parametric \a range along the \a lane.
 */
physics::Speed getMaxSpeed(Lane const &lane,
                           physics::ParametricRange const &range);

/**
 * @returns Speed limits on the provided parametric \a range along the \a lane.
 */
restriction::SpeedLimitList getSpeedLimits(
    Lane const &lane, physics::ParametricRange const &range);

/**
 * @return minimum duration to drive the provided parametric \a range along the
 * \a lane based on the speed limits of the lane.
 */
physics::Duration getDuration(Lane const &lane,
                              physics::ParametricRange const &range);

/**
 * @returns Maximum of HOV restriction for the \a lane
 */
inline restriction::PassengerCount getHOV(Lane const &lane) {
  return getHOV(lane.restrictions);
}

/**
 * @brief Find contact lanes at specific relative location.
 * @param[in] lane the lane.
 * @param[in] location Location of interest.
 * @returns Contacts to lane at specified location.
 */
ContactLaneList getContactLanes(Lane const &lane,
                                ContactLocation const &location);

/**
 * @brief Find contact lanes at specific relative locations.
 * @param[in] locations Locations of interest.
 * @returns Contacts to lanes at specified locations.
 */
ContactLaneList getContactLanes(Lane const &lane,
                                ContactLocationList const &locations);

/**
 * @brief Check nature of contact between this lane and another lane.
 * @param[in] to_lane_id Another lane identifier.
 * @returns Type of the contact. Can be INVALID if there is no contact between
 * lanes.
 */
ContactLocation getContactLocation(Lane const &lane, LaneId const &to_lane_id);

/**
 * @returns true if direction is POSITIVE or BIDIRECTIONAL.
 */
inline bool isLaneDirectionPositive(Lane const &lane) {
  return lane.direction == LaneDirection::POSITIVE ||
         lane.direction == LaneDirection::BIDIRECTIONAL;
}

/**
 * @returns true if direction is NEGATIVE or BIDIRECTIONAL.
 */
inline bool isLaneDirectionNegative(Lane const &lane) {
  return lane.direction == LaneDirection::NEGATIVE ||
         lane.direction == LaneDirection::BIDIRECTIONAL;
}

/**
 * @returns true if lane can be used in Routing.
 */
inline bool isRouteable(Lane const &lane) {
  return lane.type == LaneType::INTERSECTION || lane.type == LaneType::MULTI ||
         lane.type == LaneType::NORMAL || lane.type == LaneType::TURN;
}

/**
 * @returns true if lane is part of an intersection.
 */
inline bool isLanePartOfAnIntersection(Lane const &lane) {
  return (lane.type == lane::LaneType::INTERSECTION);
}

/**
 * @returns true if there are no lanes left of this one.
 */
inline bool isLeftMost(Lane const &lane) {
  return getContactLanes(lane, ContactLocation::LEFT).empty();
}

/**
 * @returns true if there are no lanes right of this one.
 */
inline bool isRightMost(Lane const &lane) {
  return getContactLanes(lane, ContactLocation::RIGHT).empty();
}

/**
 * @brief Calculates parametric point on the lane.
 *        Point is calculated by first calculating t_long parametric point
 *        on the left and on the right boundary, and then parametric point
 *        t_lat between those two points.
 * @param[in] t_long Longitudinal parameter.
 * @param[in] t_lat  Lateral parameter relative to the left edge.
 * @return Parametric point on the lane. Can be invalid.
 */
point::ECEFPoint getParametricPoint(
    Lane const &lane, physics::ParametricValue const &longitudinalOffset,
    physics::ParametricValue const &lateralOffset);

/**
 * @brief Calculates the projection on left and right boundary of the given
 * referencePoint.
 * @param[in] referencePoint reference point.
 * @param[out] point_on_left_edge projected point on the left boundary
 * @param[out] point_on_right_edge projected point on the right boundary
 * @return True if the operation succeeded.
 */
bool projectParametricPointToEdges(Lane const &lane,
                                   point::ECEFPoint const &referencePoint,
                                   point::ECEFPoint &point_on_left_edge,
                                   point::ECEFPoint &point_on_right_edge);

/**
 * @brief Calculates the projection on left and right boundary of the parametric
 * point on the center of the lane.
 * @param[in] longitudinalOffset Longitudinal parameter on the lane.
 * @param[out] point_on_left_edge projected point on the left boundary
 * @param[out] point_on_right_edge projected point on the right boundary
 * @return True if the operation succeeded.
 */
bool projectParametricPointToEdges(
    Lane const &lane, physics::ParametricValue const &longitudinalOffset,
    point::ECEFPoint &point_on_left_edge,
    point::ECEFPoint &point_on_right_edge);

/**
 * @brief Calculates projected parametric point on the lane.
 *        Point is calculated by first calculating the projected points on left
 * and right boundary using ProjectParametricPointToEdges(), and then parametric
 * point t_lat between those two points.
 * @param[in] longitudinalOffset Longitudinal parameter.
 * @param[in] lateralOffset  Lateral parameter relative to the left edge.
 * @return Parametric point on the lane. Can be invalid.
 */
point::ECEFPoint getProjectedParametricPoint(
    Lane const &lane, physics::ParametricValue const &longitudinalOffset,
    physics::ParametricValue const &lateralOffset);

/**
 * @returns Middle of the lane at the start.
 */
inline point::ECEFPoint getStartPoint(Lane const &lane) {
  return getParametricPoint(lane, physics::ParametricValue(0.),
                            physics::ParametricValue(0.5));
}

/**
 * @returns Middle of the lane at the end.
 */
inline point::ECEFPoint getEndPoint(Lane const &lane) {
  return getParametricPoint(lane, physics::ParametricValue(1.),
                            physics::ParametricValue(0.5));
}

/**
 * @brief Checks if Lane is longitudinally connected with another Lane at the
 * end.
 * @param[in] other Other object. Must be IsValid()!
 * @returns True if this Lane longitudinally connected with another Lane at the
 * end.
 */
bool isPyhsicalSuccessor(Lane const &lane, Lane const &other);

/**
 * @brief Checks if Lane is longitudinally connected with another Lane at the
 * start.
 * @param[in] other Other object. Must be IsValid()!
 * @returns True if this Lane longitudinally connected with another Lane at the
 * start.
 */
bool isPhysicalPredecessor(Lane const &lane, Lane const &other);

/**
 * @returns true if this lane is vanishing at the start.
 */
bool isVanishingLaneStart(Lane const &lane);

/**
 * @returns true if this lane is vanishing at the end.
 */
bool isVanishingLaneEnd(Lane const &lane);

/**
 * @brief Checks if Lane satisfies filter condition.
 * @param[in] typeFilter    Type of the lane as string.
 * @param[in] isHov         True if only lanes with HOV restriction shall be
 * returned.
 * @returns true if Lane satisfies filter condition.
 */
bool satisfiesFilter(Lane const &lane, std::string const &typeFilter,
                     bool isHov);

/**
 * @brief get the neighborhood relation to the other lane
 *
 * @param[in] laneId the main lane
 * @param[in] checkLaneId the lane to be checked on the
 *
 * @retval lane::ContactLocation::OVERLAP: if it's the same lane
 * @retval lane::ContactLocation::LEFT: if it's a left neighbor
 * @retval lane::ContactLocation::RIGHT: if it's a right neighbor
 * @retval lane::ContactLocation::SUCCESSOR: if it a successor lane
 * @retval lane::ContactLocation::PREDECESSOR: if it's a predecessor lane
 * @retval lane::ContactLocation::INVALID: if it's nothing of the above
 */
ContactLocation getDirectNeighborhoodRelation(LaneId const laneId,
                                              LaneId const checkLaneId);

/**
 * @brief Check if a lane is successor or predecessor of a lane
 *
 * @param[in] laneId the main lane
 * @param[in] checkLaneId the lane to be checked if it's a successor or
 * predecessor of the main lane
 *
 * @returns \c true if the \a checkLaneId is a successor or predecessor of \a
 * laneId
 */
bool isSuccessorOrPredecessor(LaneId const laneId, LaneId const checkLaneId);

/**
 * @brief Check if two lanes are direct neighbors of even the same
 *
 * @returns \c true if the ids belong to the same or neighboring lanes, false
 * otherwise
 */
bool isSameOrDirectNeighbor(LaneId const id, LaneId const neighbor);

/**
 * @brief Check if the lane direction is positive
 *
 * @param[in] laneId the id of the lane to be checked
 *
 * @returns true if the lane direction is positive (positive || bidirectional)
 */
bool isLaneDirectionPositive(LaneId const &laneId);

/**
 * @brief Check if the lane direction is negative
 *
 * @param[in] laneId the id of the lane to be checked
 *
 * @returns true if the lane direction is negative (negative || bidirectional)
 */
bool isLaneDirectionNegative(LaneId const &laneId);

/**
 * @brief Get the distance of an object to the lane boundaries
 *
 * If the object provides a map matched bounding box, this information is used
 * first (speed-up and more accurate). If the object's map matched bounding box
 * is not touching the \c laneId at all, the object's center distance to the
 * lane is calculated and reduced by max(width, length)/2 to obtain a good
 * estimate for the shortest distance to the lane.
 */
physics::Distance getDistanceToLane(LaneId laneId, match::Object const &object);

/**
 * @return the ENU heading of the lane at the given position
 */
point::ENUHeading getLaneENUHeading(point::ParaPoint const &position);

/**
 * @brief Check if the provided heading is along the lane direction
 *
 * @param[in] position is the position at which the lane heading is obtained
 * @param[in] heading is the heading to be checked
 *
 * @return true if the heading is within 90° of the lane direction, false
 * otherwise
 */
bool isHeadingInLaneDirection(point::ParaPoint const &position,
                              point::ENUHeading const &heading);

/**
 * @brief project a position to a neighboring lane (incl. the lane itself),
 * which has the provided heading
 *
 * @param[in]  position to be projected
 * @param[in]  heading to be checked
 * @param[out] projectedPosition is the projected position (can be position, if
 * the lane is already in heading direction)
 *
 * @return true if a valid projection was found, false otherwise
 */
bool projectPositionToLaneInHeadingDirection(
    point::ParaPoint const &position, point::ENUHeading const &heading,
    point::ParaPoint &projectedPosition);

/**
 * @brief Finds point on the lane nearest to the given point.
 * @param[in] lane the lane.
 * @param[in] pt Given point.
 * @param[out] mmpos Resulting map-matched position.
 *   The resulting probability of the map matched position is derived from
 * lateralT value: spans between [0.5; 1.0] for LANE_IN matches spans between
 * [0.1; 0.5] for LANE_RIGHT/LANE_LEFT matches
 *
 * @returns true if successful.
 */
bool findNearestPointOnLane(Lane const &lane, point::ECEFPoint const &pt,
                            match::MapMatchedPosition &mmpos);

/**
 * @brief Finds point on the lane interval nearest to the given point.
 * @param[in] laneInterval the interval of the lane to be considered.
 * @param[in] pt Given point.
 * @param[out] mmpos Resulting map-matched position.
 *   The resulting probability of the map matched position is derived from
 * lateralT value: spans between [0.5; 1.0] for LANE_IN matches spans between
 * [0.1; 0.5] for LANE_RIGHT/LANE_LEFT matches
 *
 * @returns true if successful.
 */
bool findNearestPointOnLaneInterval(route::LaneInterval const &laneInterval,
                                    point::ECEFPoint const &pt,
                                    match::MapMatchedPosition &mmpos);

/**
 * @brief get the ENU point of a lane
 *
 */
point::ENUPoint getENULanePoint(point::ParaPoint const parametricPoint,
                                physics::ParametricValue const &lateralOffset =
                                    physics::ParametricValue(0.5));

/** @brief calculate length of a lane with a given id
 */
physics::Distance calcLength(LaneId const &laneId);

/** @brief calculate length of a lane occupied region
 */
physics::Distance calcLength(
    match::LaneOccupiedRegion const &laneOccupiedRegion);

/** @brief calculate the width of the lane at the provided lanePoint
 */
physics::Distance calcWidth(point::ParaPoint const &paraPoint);

/** @brief calculate the width of the provided lane at the provided longitudinal
 * position
 */
physics::Distance calcWidth(LaneId const &laneId,
                            physics::ParametricValue const &longOffset);

/**
 * @brief calculate the width of a lane at the provided ENU position
 *
 * @return -1 in case the enuPoint could not be found on a lane
 */
physics::Distance calcWidth(point::ENUPoint const &enuPoint);

/** @brief calculate width of a lane occupied region
 *  scales the with at the center of the occupied region by the lateral extend
 * of the region
 */
physics::Distance calcWidth(
    match::LaneOccupiedRegion const &laneOccupiedRegion);

}  // namespace lane
}  // namespace map
}  // namespace ad
