// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/access/Types.hpp"
#include "ad/map/lane/Types.hpp"
#include "ad/map/route/Types.hpp"

/* @brief namespace ad */
namespace ad {
/* @brief namespace map */
namespace map {
/* @brief namespace route */
namespace route {

/**
 * @brief class defining a route interval with one lane
 *
 * The route interval consists of the id of the lane, the start and end point of
 * lane on the route as TParam. TParam describes a parametric point along the
 * lane geometry. By defining the start and end of the route interval, also a
 * direction of the route is defined ... at least if @c start and @c end points
 * differ from each other.
 *
 * If start == end, the route interval is degenerated and therefore no
 * meaningful route direction can be derived. Only if start != end, checks like
 * isAfterInterval(), isBeforeInterval(), isRouteDirectionPositive(),
 * isRouteDirectionNegative() provide the expected information.
 */

/**
 * @brief get interval start as ParaPoint
 */
inline point::ParaPoint getIntervalStart(LaneInterval const &laneInterval) {
  point::ParaPoint result;
  result.laneId = laneInterval.laneId;
  result.parametricOffset = laneInterval.start;
  return result;
}

/**
 * @brief get interval start as ParaPoint for a given laneId and a given route
 *
 * throws if laneId not found in route
 */
point::ParaPoint getIntervalStart(FullRoute const &route,
                                  lane::LaneId const &laneId);

/**
 * @brief get interval end as ParaPoint
 */
inline point::ParaPoint getIntervalEnd(LaneInterval const &laneInterval) {
  point::ParaPoint result;
  result.laneId = laneInterval.laneId;
  result.parametricOffset = laneInterval.end;
  return result;
}

/**
 * @brief convert lane interval to ParametricRange
 */
inline physics::ParametricRange toParametricRange(
    route::LaneInterval const &laneInterval) {
  physics::ParametricRange range;
  if (laneInterval.start <= laneInterval.end) {
    range.minimum = laneInterval.start;
    range.maximum = laneInterval.end;
  } else {
    range.minimum = laneInterval.end;
    range.maximum = laneInterval.start;
  }
  return range;
}

/**
 * @brief get the signed parametric physics::Distance between two parametric
 * points respecting the laneInterval's direction
 *
 * @retval > 0, if the \c first point is before the \c second point within the
 * interval
 * @retval < 0, if the \c second point is before the \c first point within the
 * interval
 * @retval = 0, if both are equal
 *
 * @throws std::invalid_argument if the parametric points are not referring to
 * the landId
 */
physics::ParametricValue getSignedDistance(LaneInterval const &laneInterval,
                                           point::ParaPoint const &first,
                                           point::ParaPoint const &second);

/**
 * @brief get the unsigned parametric physics::Distance between two parametric
 * points
 *
 * @retval | \c first.parametricOffset - \c second.parametricOffset |
 * @throws std::invalid_argument if the parametric points are not referring to
 * the landId
 */
physics::ParametricValue getUnsignedDistance(LaneInterval const &laneInterval,
                                             point::ParaPoint const &first,
                                             point::ParaPoint const &second);

/**
 * @brief checks if the point marks the start of the interval
 *
 * @param[in] point parametric point to be checked against the route interval
 * start
 *
 * @returns @c true if LaneId and TParam at start of this matches the given @c
 * point
 */
inline bool isStartOfInterval(LaneInterval const &laneInterval,
                              point::ParaPoint const &point) {
  return (point.laneId == laneInterval.laneId) &&
         (point.parametricOffset == laneInterval.start);
}

/**
 * @brief checks if the point marks the end of the interval
 *
 * @param[in] point parametric point to be checked against the route interval
 * end
 *
 * @returns @c true if LaneId and TParam at end of this matches the given @c
 * point
 */
inline bool isEndOfInterval(LaneInterval const &laneInterval,
                            point::ParaPoint const &point) {
  return (point.laneId == laneInterval.laneId) &&
         (point.parametricOffset == laneInterval.end);
}

/**
 * @brief checks if the direction of this route interval is positive in respect
 * to the lane geometry
 *
 * @returns @c true if the parametric points follow: start < end
 * @returns @c lane::isLaneDirectionPositive(laneInterval.laneId) ^ wrongWay if
 * start == end  (degenerated interval uses wrong way flag to determine the
 * route direction)
 */
bool isRouteDirectionPositive(LaneInterval const &laneInterval);

/**
 * @brief checks if the direction of this route interval is negative in respect
 * to the lane geometry
 *
 * @returns !isRouteDirectionPositive()
 */
inline bool isRouteDirectionNegative(LaneInterval const &laneInterval) {
  return !isRouteDirectionPositive(laneInterval);
}

/**
 * @brief checks if the route direction is aligned with the nominal driving
 * direction of the lane
 */
bool isRouteDirectionAlignedWithDrivingDirection(
    LaneInterval const &laneInterval);

/**
 * @brief checks if the parametric offset is within the interval
 *
 * @param[in] parametricOffset parametric offset to be checked against the route
 * interval
 *
 * @returns @c true if parametric offset  is within the interval
 */
inline bool isWithinInterval(LaneInterval const &laneInterval,
                             physics::ParametricValue const &parametricOffset) {
  if (isRouteDirectionPositive(laneInterval)) {
    return (laneInterval.start <= parametricOffset) &&
           (parametricOffset <= laneInterval.end);
  } else {
    return (laneInterval.end <= parametricOffset) &&
           (parametricOffset <= laneInterval.start);
  }
}

/**
 * @brief checks if the point is within the interval
 *
 * @param[in] point parametric point to be checked against the route interval
 *
 * @returns @c true if LaneId matches and TParam of the given @c point is within
 * the interval
 */
inline bool isWithinInterval(LaneInterval const &laneInterval,
                             point::ParaPoint const &point) {
  return (point.laneId == laneInterval.laneId) &&
         isWithinInterval(laneInterval, point.parametricOffset);
}

/**
 * @brief checks if the interval is degenerated
 *
 * The interval is degenerated if start and end of the interval are equal.
 * In such a case no meaningful information on route direction is possible.
 *
 * @returns @c true if the interval is degenerated: start == end
 */
inline bool isDegenerated(LaneInterval const &laneInterval) {
  return (laneInterval.start == laneInterval.end);
}

/**
 * @brief checks if the  parametric offset is after the interval
 *
 * @param[in] parametricOffset parametric offset to be checked against the route
 * interval
 *
 * @returns @c true if given parametric offset is after the interval
 * Be aware: if the route interval is degenerated isAfterInterval() and
 * isBeforeInterval() return the same value
 */
inline bool isAfterInterval(LaneInterval const &laneInterval,
                            physics::ParametricValue const parametricOffset) {
  if (isRouteDirectionPositive(laneInterval)) {
    return (parametricOffset > laneInterval.end);
  } else {
    return (parametricOffset < laneInterval.end);
  }
}

/**
 * @brief checks if the point is after the interval
 *
 * @param[in] point parametric point to be checked against the route interval
 *
 * @returns @c true if LaneId matches and TParam of the given @c point is after
 * the interval Be aware: if the route interval is degenerated isAfterInterval()
 * and isBeforeInterval() return the same value ( point != end )
 */
inline bool isAfterInterval(LaneInterval const &laneInterval,
                            point::ParaPoint const &point) {
  if (point.laneId == laneInterval.laneId) {
    return isAfterInterval(laneInterval, point.parametricOffset);
  }
  return false;
}

/**
 * @brief checks if the parametric offset is before the interval
 *
 * @param[in] parametricOffset parametric offset to be checked against the route
 * interval
 *
 * @returns @c true if parametric offset is before the interval
 * Be aware: if the route interval is degenerated isAfterInterval() and
 * isBeforeInterval() return the same value
 */
inline bool isBeforeInterval(LaneInterval const &laneInterval,
                             physics::ParametricValue const parametricOffset) {
  if (isRouteDirectionPositive(laneInterval)) {
    return (parametricOffset < laneInterval.start);
  } else {
    return (parametricOffset > laneInterval.start);
  }
}

/**
 * @brief checks if the point is before the interval
 *
 * @param[in] point parametric point to be checked against the route interval
 *
 * @returns @c true if LaneId matches and TParam of the given @c point is before
 * the interval Be aware: if the route interval is degenerated isAfterInterval()
 * and isBeforeInterval() return the same value ( point != start )
 */
inline bool isBeforeInterval(LaneInterval const &laneInterval,
                             point::ParaPoint const &point) {
  if (point.laneId == laneInterval.laneId) {
    return isBeforeInterval(laneInterval, point.parametricOffset);
  }
  return false;
}

/**
 * @brief checks if the range overlaps with the interval
 *
 * @param[in] range parametric range to be checked against the route interval
 *
 * @returns @c true if range overlaps with the interval
 */
inline bool overlapsInterval(LaneInterval const &laneInterval,
                             physics::ParametricRange const &range) {
  bool rangeIsOutside = (isBeforeInterval(laneInterval, range.minimum) &&
                         isBeforeInterval(laneInterval, range.maximum)) ||
                        (isAfterInterval(laneInterval, range.minimum) &&
                         isAfterInterval(laneInterval, range.maximum));
  return !rangeIsOutside;
}

/**
 * @brief project a given parametric position (offset) to the center line of the
 * given neighbor lane
 *
 * @param[in] currentInterval is the current lane
 * @param[in] neighborInterval is the neighbor lane
 * @param[in] parametricOffset is the current position that should be projected
 * to the neighbor lane
 *
 * Note: If the given neighborInterval is not a neighbor of the currentInterval
 * the function will throw
 *
 * @return Projected position as parametric offset
 */
physics::ParametricValue getProjectedParametricOffsetOnNeighborLane(
    LaneInterval const &currentInterval, LaneInterval const &neighborInterval,
    physics::ParametricValue const &parametricOffset);

/**
 * @brief calculate the length of the provided lane interval as parametric value
 */
inline physics::ParametricValue calcParametricLength(
    LaneInterval const &laneInterval) {
  return std::fabs(laneInterval.start - laneInterval.end);
}

/** @brief calculate the length of the provided lane interval as
 * physics::Distance value
 *
 * For length calculation the parametric length of the interval
 * (calcParametricLength()) is multiplied by the lane's length.
 */
physics::Distance calcLength(LaneInterval const &laneInterval);

/** @brief calculate the Duration of the provided lane interval as duration
 * value
 *
 * For Duration calculations maximum allowed speed combined with the length is
 * taken into account
 */
physics::Duration calcDuration(LaneInterval const &laneInterval);

/**
 * @brief get right edge of the lane interval as ENUEdge
 *
 * Be aware: Right is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical left edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
void getRightEdge(LaneInterval const &laneInterval, point::ENUEdge &enuEdge);

/**
 * @brief get right edge of the lane interval as ECEFEdge
 *
 * Be aware: Right is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical left edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
void getRightEdge(LaneInterval const &laneInterval, point::ECEFEdge &ecefEdge);

/**
 * @brief get right edge of the lane interval as GeoEdge
 *
 * Be aware: Right is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical left edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
void getRightEdge(LaneInterval const &laneInterval, point::GeoEdge &geoEdge);

/**
 * @brief get right edge of the lane interval as ENUEdge
 *
 * Be aware: Right is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical left edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
point::ENUEdge getRightENUEdge(LaneInterval const &laneInterval);

/**
 * @brief get right edge of the lane interval as ECEFEdge
 *
 * Be aware: Right is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical left edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
point::ECEFEdge getRightECEFEdge(LaneInterval const &laneInterval);

/**
 * @brief get right edge of the lane interval as GeoEdge
 *
 * Be aware: Right is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical left edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
point::GeoEdge getRightGeoEdge(LaneInterval const &laneInterval);

/**
 * @brief get right edge of the lane interval as ENUEdge using projection to
 * find the start of the edge
 *
 * Be aware: Right is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical left edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
void getRightProjectedEdge(LaneInterval const &laneInterval,
                           point::ENUEdge &enuEdge);

/**
 * @brief get right edge of the lane interval as ECEFEdge using projection to
 * find the start of the edge
 *
 * Be aware: Right is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical left edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
void getRightProjectedEdge(LaneInterval const &laneInterval,
                           point::ECEFEdge &ecefEdge);

/**
 * @brief get right edge of the lane interval as GeoEdge using projection to
 * find the start of the edge
 *
 * Be aware: Right is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical left edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
void getRightProjectedEdge(LaneInterval const &laneInterval,
                           point::GeoEdge &geoEdge);

/**
 * @brief get right edge of the lane interval as ENUEdge using projection to
 * find the start of the edge
 *
 * Be aware: Right is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical left edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
point::ENUEdge getRightProjectedENUEdge(LaneInterval const &laneInterval);

/**
 * @brief get right edge of the lane interval as ECEFEdge using projection to
 * find the start of the edge
 *
 * Be aware: Right is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical left edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
point::ECEFEdge getRightProjectedECEFEdge(LaneInterval const &laneInterval);

/**
 * @brief get right edge of the lane interval as GeoEdge using projection to
 * find the start of the edge
 *
 * Be aware: Right is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical left edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
point::GeoEdge getRightProjectedGeoEdge(LaneInterval const &laneInterval);

/**
 * @brief get Left edge of the lane interval as ENUEdge
 *
 * Be aware: Left is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical right edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
void getLeftEdge(LaneInterval const &laneInterval, point::ENUEdge &enuEdge);

/**
 * @brief get Left edge of the lane interval as ECEFEdge
 *
 * Be aware: Left is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical right edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
void getLeftEdge(LaneInterval const &laneInterval, point::ECEFEdge &ecefEdge);

/**
 * @brief get Left edge of the lane interval as GeoEdge
 *
 * Be aware: Left is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical right edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
void getLeftEdge(LaneInterval const &laneInterval, point::GeoEdge &geoEdge);

/**
 * @brief get left edge of the lane interval as ENUEdge
 *
 * Be aware: Left is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical right edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
point::ENUEdge getLeftENUEdge(LaneInterval const &laneInterval);

/**
 * @brief get left edge of the lane interval as ECEFEdge
 *
 * Be aware: Left is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical right edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
point::ECEFEdge getLeftECEFEdge(LaneInterval const &laneInterval);

/**
 * @brief get left edge of the lane interval as GeoEdge
 *
 * Be aware: Left is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical right edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
point::GeoEdge getLeftGeoEdge(LaneInterval const &laneInterval);

/**
 * @brief get Left edge of the lane interval as ENUEdge using projection to find
 * the start of the edge
 *
 * Be aware: Left is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical right edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
void getLeftProjectedEdge(LaneInterval const &laneInterval,
                          point::ENUEdge &enuEdge);

/**
 * @brief get Left edge of the lane interval as ECEFEdge using projection to
 * find the start of the edge
 *
 * Be aware: Left is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical right edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
void getLeftProjectedEdge(LaneInterval const &laneInterval,
                          point::ECEFEdge &ecefEdge);

/**
 * @brief get Left edge of the lane interval as GeoEdge using projection to find
 * the start of the edge
 *
 * Be aware: Left is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical right edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
void getLeftProjectedEdge(LaneInterval const &laneInterval,
                          point::GeoEdge &geoEdge);

/**
 * @brief get left edge of the lane interval as ENUEdge using projection to find
 * the start of the edge
 *
 * Be aware: Left is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical right edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
point::ENUEdge getLeftProjectedENUEdge(LaneInterval const &laneInterval);

/**
 * @brief get left edge of the lane interval as ECEFEdge using projection to
 * find the start of the edge
 *
 * Be aware: Left is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical right edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
point::ECEFEdge getLeftProjectedECEFEdge(LaneInterval const &laneInterval);

/**
 * @brief get left edge of the lane interval as GeoEdge using projection to find
 * the start of the edge
 *
 * Be aware: Left is in the sense of the lane interval orientation.
 *    If the lane interval has negative direction the physical right edge of the
 * underlying lane is returned. Furthermore, the points are ordered in the
 * logical lane interval direction: the first point is at lane interval start
 * and the last one at lane interval end.
 */
point::GeoEdge getLeftProjectedGeoEdge(LaneInterval const &laneInterval);

/**
 * @brief get the geo borders of this lane
 */
lane::GeoBorder getGeoBorder(LaneInterval const &laneInterval);

/**
 * @brief get the ecef borders of this lane
 */
lane::ECEFBorder getECEFBorder(LaneInterval const &laneInterval);

/**
 * @brief get the enu borders of this lane
 *
 * Note: The border (left/right edge) will start with the parametric offset
 * given by the laneInterval.
 */
lane::ENUBorder getENUBorder(LaneInterval const &laneInterval);

/**
 * @brief get the enu borders of this lane
 *
 * Note: The border (left/right edge) does not necessarily start/end with the
 * parametric offset given by the laneInterval. Instead this offset is used
 * for the center line and the resulting point is orthogonally projected on
 * the edges to obtain the border
 */
lane::ENUBorder getENUProjectedBorder(LaneInterval const &laneInterval);

/**
 * @brief shorten the LaneInterval about a given physics::Distance.
 * Will remove at the begin of the LaneInterval
 *
 * @param[in] the LaneInterval that should be shortened
 * @param[in] physics::Distance the LaneInterval should be shortened
 *
 * @returns resulting shortened LaneInterval
 *
 * If physics::Distance is bigger then the length of the interval. Resulting
 * length will be zero. LaneInterval.start = LaneInterval.end
 */
LaneInterval shortenIntervalFromBegin(LaneInterval const &laneInterval,
                                      physics::Distance const &distance);

/**
 * @brief shorten the LaneInterval about a given physics::Distance.
 * Will remove at the end of the LaneInterval
 *
 * @param[in] laneInterval the LaneInterval that should be shortened
 * @param[in] physics::Distance the LaneInterval should be shortened
 *
 * @returns resulting shortened LaneInterval
 *
 * If physics::Distance is bigger then the length of the interval. Resulting
 * length will be zero. LaneInterval.end = LaneInterval.start
 */
LaneInterval shortenIntervalFromEnd(LaneInterval const &laneInterval,
                                    physics::Distance const &distance);

/**
 * @brief Restrict length of the LaneInterval to a given physics::Distance from
 * start.
 *
 * @param[in] laneInterval the LaneInterval that should be restricted
 * @param[in] physics::Distance the LaneInterval should be restricted to
 *
 * @returns resulting shortened LaneInterval
 *
 * If physics::Distance is bigger then the length of the interval. Resulting
 * LaneInterval is the input Interval.
 */
LaneInterval restrictIntervalFromBegin(LaneInterval const &laneInterval,
                                       physics::Distance const &distance);

/**
 * @brief extend the lane interval until the end of the lane reached
 *
 * The end is set to the lane's maximal value (depending on the route direction
 * to 0. or 1.). If the lane interval is degenerated nothing is done.
 *
 * @param[in] laneInterval the LaneInterval that should be extended
 *
 * @returns resulting extended LaneInterval
 */
LaneInterval extendIntervalUntilEnd(LaneInterval const &laneInterval);

/**
 * @brief cut the LaneInterval at a given parametric point.
 * Will remove at the start of the LaneInterval
 *
 * @param[in] the LaneInterval that should be shortened
 * @param[in] newIntervalStart the new start of the interval if the point is
 * within the LaneInterval
 *
 * @returns resulting shortened LaneInterval
 *
 * If the parametric offset is outside of the interval, the original lane
 * interval is returned.
 */
LaneInterval cutIntervalAtStart(
    LaneInterval const &laneInterval,
    physics::ParametricValue const &newIntervalStart);

/**
 * @brief extend the lane interval until the start of the lane reached
 *
 * The start is set to the lane's minimal value (depending on the route
 * direction to 1. or 0.). If the lane interval is degenerated nothing is done.
 *
 * @param[in] laneInterval the LaneInterval that should be extended
 *
 * @returns resulting extended LaneInterval
 */
LaneInterval extendIntervalUntilStart(LaneInterval const &laneInterval);

/**
 * @brief extend the lane interval by moving its start position by
 * physics::Distance
 *
 * If the lane interval is degenerated nothing is done.
 *
 * @param[in] laneInterval the LaneInterval that should be extended
 * @param[in] physics::Distance by which the LaneInterval should be extended
 *
 * @returns resulting extended LaneInterval
 */
LaneInterval extendIntervalFromStart(LaneInterval const &laneInterval,
                                     physics::Distance const &distance);

/**
 * @brief extend the lane interval by moving its end position by
 * physics::Distance
 *
 * If the lane interval is degenerated nothing is done.
 *
 * @param[in] laneInterval the LaneInterval that should be extended
 * @param[in] physics::Distance by which the LaneInterval should be extended
 *
 * @returns resulting extended LaneInterval
 */
LaneInterval extendIntervalFromEnd(LaneInterval const &laneInterval,
                                   physics::Distance const &distance);

/**
 * @brief cut the LaneInterval at a given parametric point.
 * Will remove at the end of the LaneInterval
 *
 * @param[in] the LaneInterval that should be shortened
 * @param[in] newIntervalEnd the new end of the interval if the point is within
 * the LaneInterval
 *
 * @returns resulting shortened LaneInterval
 *
 * If the parametric offset is outside of the interval, the original lane
 * interval is returned.
 */
LaneInterval cutIntervalAtEnd(LaneInterval const &laneInterval,
                              physics::ParametricValue const &newIntervalEnd);

/**
 * @brief get the speed limits of the lane interval
 */
restriction::SpeedLimitList getSpeedLimits(LaneInterval const &laneInterval);

/**
 * @brief get the metric ranges of the lane interval
 */
void getMetricRanges(LaneInterval const &laneInterval,
                     physics::MetricRange &lengthRange,
                     physics::MetricRange &widthRange);

}  // namespace route
}  // namespace map
}  // namespace ad
