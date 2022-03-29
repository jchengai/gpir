// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/lane/Types.hpp"
#include "ad/map/match/Types.hpp"
#include "ad/map/point/Types.hpp"
#include "ad/map/route/LaneIntervalOperation.hpp"
#include "ad/map/route/Routing.hpp"
#include "ad/map/route/Types.hpp"
#include "ad/physics/Types.hpp"

/* @brief namespace ad */
namespace ad {
/* @brief namespace map */
namespace map {

/** @brief namespace intersection */
namespace intersection {
/**
 * @brief forward declaration of Intersection class
 */
class Intersection;
}

/* @brief namespace route */
namespace route {

/**
 * @brief struct defining iterator information on a route
 */
struct RouteIterator
{
  RouteIterator(FullRoute const &iRoute, RoadSegmentList::const_iterator const &iRoadSegmentIterator)
    : route(iRoute)
    , roadSegmentIterator(iRoadSegmentIterator)
  {
  }

  /**
   * @brief check for validity of the iterator
   */
  bool isValid() const
  {
    return roadSegmentIterator != std::end(route.roadSegments);
  }

  /** @brief reference to the route */
  FullRoute const &route;
  /** @brief iterator referencing an element of the queryRoute */
  RoadSegmentList::const_iterator roadSegmentIterator;
};

/**
 * @brief get the route iterator from a given route position
 *
 * @param[in] routePosition the route position to find
 * @param[in] route the route to check for the position
 *
 * @returns the RouteIterator pointing to the corresponding route segment.
 *  If the routePosition is not found an invalid RouteIterator is returned.
 */
RouteIterator getRouteIterator(route::RouteParaPoint const &routePosition, route::FullRoute const &route);

/**
 * @brief get the lane para points in respect to the given route parametric offset
 *
 * @param[in] routeParametricOffset the parametric offset in respect to the route
 * @param[in] laneInterval the laneInterval to consider, the whole lane interval is considered to be a full segment
 *   of a route.
 *
 * @returns the parametric point within the corresponding lane interval in respect to the given route parametric offset.
 */
point::ParaPoint getLaneParaPoint(physics::ParametricValue const &routeParametricOffset,
                                  route::LaneInterval const &laneInterval);

/**
 * @brief get the signed distance to a lane w.r.t to a given route (direction) for the map matched positions
 *
 * This check searches the mapMatchedPositions for the given \a checkLaneId and returns a signed distance value to the
 * lane.
 *
 * @param[in] checkLaneId the lane id to find
 * @param[in] route in which the lane should be in
 * @param[in] mapMatchedPositions the map matched positions to check
 *
 * @returns:
 * If not part of the mapMatchedPositions:  distance = std::numeric_limits<physics::Distance>::max()
 * If it's found and map matched type is:
 * - map matched position is on the lane: distance = 0
 * - map matched position is left of lane: distance < 0
 * - map matched position is right of lane: distance > 0
 *
 * @throws std::runtime_error if the lane is not found inside the route
 */
physics::Distance signedDistanceToLane(lane::LaneId const &checkLaneId,
                                       FullRoute const &route,
                                       match::MapMatchedPositionConfidenceList const &mapMatchedPositions);

/**
 * @brief get the lane para points in respect to the given route position
 *
 * @param[in] routePosition the route position to find
 * @param[in] route the route to check for the position
 *
 * @returns the lane segment para points within the corresponding route segment.
 * If the routePosition is not found an empty list is returned.
 */
point::ParaPointList getLaneParaPoints(route::RouteParaPoint const &routePosition, route::FullRoute const &route);

/** @brief calculate the length of the provided full route
 */
physics::Distance calcLength(FullRoute const &fullRoute);

/** @brief calculate the length of the provided road segment
 *
 * For length calculations the shortest road segment of this is used as reference.
 */
physics::Distance calcLength(RoadSegment const &roadSegment);

/** @brief calculate the length of the provided lane segment
 */
inline physics::Distance calcLength(LaneSegment const &laneSegment)
{
  return calcLength(laneSegment.laneInterval);
}

/** @brief calculate the length of the provided connecting route
 *
 * For length calculations the longest route section of this is used as reference.
 */
physics::Distance calcLength(ConnectingRoute const &connectingRoute);

/** @brief calculate the length between the two given iterators
 */
physics::Distance calcLength(RouteIterator const &startIterator, RouteIterator const &endIterator);

physics::Distance
calcLength(RouteParaPoint const &startRouteParaPoint, RouteParaPoint const &endRouteParaPoint, FullRoute const &route);

/** @brief calculate the Duration of the provided full route
 */
physics::Duration calcDuration(FullRoute const &fullRoute);

/** @brief calculate the Duration of the provided road segment
 *
 * For Duration calculations the shortest road segment of this is used as reference.
 */
physics::Duration calcDuration(RoadSegment const &roadSegment);

/** @brief calculate the Duration of the provided lane segment
 */
inline physics::Duration calcDuration(LaneSegment const &laneSegment)
{
  return calcDuration(laneSegment.laneInterval);
}

/** @brief calculate the Duration of the provided connecting route
 *
 * For Duration calculations the longest route section of this is used as reference.
 */
physics::Duration calcDuration(ConnectingRoute const &connectingRoute);

/**
 * @brief get the speed limits of the road segment
 */
restriction::SpeedLimitList getSpeedLimits(RoadSegment const &roadSegment);

/**
 * @brief get the speed limits of the lane segment
 */
restriction::SpeedLimitList getSpeedLimits(LaneSegment const &laneSegment);

/**
 * @brief get the speed limits between the two given iterators
 */
restriction::SpeedLimitList getSpeedLimits(RouteIterator const &startIterator, RouteIterator const &endIterator);

/**
 * @brief get the speed limits of a complete route
 */
restriction::SpeedLimitList getSpeedLimits(FullRoute const &fullRoute);

/**
 * @brief get the speed limits of a connecting route
 */
restriction::SpeedLimitList getSpeedLimits(ConnectingRoute const &connectingRoute);

/**
 * @brief checks if the point is within the roadSegment
 *
 * @param[in] point parametric point to be checked against the road segment
 *
 * @returns \c true if the point is within one of the segments lane intervals
 */
bool isWithinInterval(RoadSegment const &roadSegment, point::ParaPoint const &point);

/**
 * @brief struct defining the result type of findWaypoint() and findNearestWaypoint()
 */
struct FindWaypointResult
{
  /**
   * @brief constructor
   *
   * constructs a result that is invalid
   */
  FindWaypointResult(FullRoute const &route);

  /**
   * @brief constructor
   *
   * constructs a result with the provided data
   */
  FindWaypointResult(FullRoute const &route,
                     point::ParaPoint const &iQueryPosition,
                     RoadSegmentList::const_iterator roadSegmentIter,
                     LaneSegmentList::const_iterator laneIntervalIter)
    : queryRoute(route)
    , roadSegmentIterator(roadSegmentIter)
    , laneSegmentIterator(laneIntervalIter)
    , queryPosition(iQueryPosition)
  {
  }

  /**
   * @brief copy constructor
   */
  FindWaypointResult(FindWaypointResult const &other) = default;

  /** @brief reference to the route */
  FullRoute const &queryRoute;
  /** @brief iterator referencing an element of the queryRoute */
  RoadSegmentList::const_iterator roadSegmentIterator;
  /** @brief iterator referencing an element of the routeIterator content; only valid if routeIterator !=
   * queryRoute.end() */
  LaneSegmentList::const_iterator laneSegmentIterator;
  /** @brief copy of the query position this waypoint result refers to */
  point::ParaPoint queryPosition;

  /**
   * @brief checks whether the FindWaypointResult contains valid data
   */
  bool isValid() const
  {
    return roadSegmentIterator != queryRoute.roadSegments.end()
      && laneSegmentIterator != roadSegmentIterator->drivableLaneSegments.end();
  }
  /**
   * @brief assignment operator
   *
   * The assignment is only possible if the other refers to the same query route as this.
   * If there is a miss match a std::invalid_argument exception is thrown
   */
  FindWaypointResult &operator=(FindWaypointResult const &other);

  /**
   * @brief return a FindWayPointResult that is iterated to the left lane of this
   *
   * If there is no left lane, the result is invalid (see isValid())
   * @throws std::runtime_error() in case of an inconsistent route
   */
  FindWaypointResult getLeftLane() const;

  /**
   * @brief return a FindWayPointResult that is iterated to the right lane of this
   *
   * If there is no right lane, the result is invalid (see isValid())
   * @throws std::runtime_error() in case of an inconsistent route
   */
  FindWaypointResult getRightLane() const;

  /**
   * @brief return a vector of FindWayPointResult that is iterated to the successor lanes of this
   *
   * If there are no successor lanes, the result is empty.
   * @throws std::runtime_error() in case of an inconsistent route
   */
  std::vector<FindWaypointResult> getSuccessorLanes() const;

  /**
   * @brief return a vector of FindWayPointResult that is iterated to the predecessor lanes of this
   *
   * If there are no predecessor lanes, the result is empty.
   * @throws std::runtime_error() in case of an inconsistent route
   */
  std::vector<FindWaypointResult> getPredecessorLanes() const;
};

/** @brief calculate the length of the route in respect to FindWaypointResult
 *
 *  calculate the length of the route up to the queryPosition of the FindWaypointResult
 */
physics::Distance calcLength(FindWaypointResult const &findWaypointResult);

/** @brief find a waypoint on the route containing the given position
 *
 *  The position is searched within the given full route. If the position's LaneId is present in the route,
 *  and the position's offset is within the route (i.e. the point lies within the route),
 *  an iterator of the route is returned in conjunction with the RouteInterval defining the interval of the route
 *  covering the corresponding lane.
 *
 *  @param[in] position the position defining the lane id which should be searched for within the route
 *  @param[in] route the route to search in
 *
 *  @returns waypoint result containing an iterator pointing to the route segment the lane of
 *   the position is part of and the corresponding route interval having that lane id.
 *   The query position is the provided \a position.
 */
FindWaypointResult findWaypoint(point::ParaPoint const &position, route::FullRoute const &route);

/** @brief find a waypoint on the route containing the given laneId
 *
 *  The position is searched within the given full route. If the position's LaneId is present in the route,
 *  an iterator of the route is returned in conjunction with the RouteInterval defining the interval of the route
 *  covering the corresponding lane.
 *
 *  @param[in] laneId the lane id which should be searched for within the route
 *  @param[in] route the route to search in
 *
 *  @returns waypoint result containing an iterator pointing to the route segment the lane of
 *   the position is part of and the corresponding route interval having that lane id.
 *   The query position is (on success) the start of the lane interval within the route.
 */
FindWaypointResult findWaypoint(lane::LaneId const &laneId, route::FullRoute const &route);

/** @brief find waypoint on the route containing the given positions
 *
 *  A version of findWaypoint() allowing the position to be a vector of positions to consider
 *  and select the result near to the begin of the route.
 *  All the given positions are searched using the findWaypoint(); therefore @see findWaypoint() for further details.
 *
 *  @param[in] positions the position vector which should be searched for within the route
 *  @param[in] route the route to search in
 *
 *  @returns waypoint result  containing an iterator pointing to the route segment the lane of
 *    the position is part of and the corresponding route interval having that lane id.
 *    If multiple are matching the one nearest the begin of the route is selected.
 */
FindWaypointResult findNearestWaypoint(point::ParaPointList const &positions, route::FullRoute const &route);

/**
 * @brief find nearest waypoint on the route containing the given mapmatched positions
 *
 * This function will return the nearest waypoint along the route
 *
 *  @param[in] mapMatchedPositions the map matched positions which should be searched for within the route
 *  @param[in] route the route to search in
 *
 *  @returns waypoint result containing an iterator pointing to the route segment the lane of
 *   the position is part of and the corresponding route interval having that lane id.
 *   If multiple are matching the one nearest the begin of the route is selected.
 */
FindWaypointResult findNearestWaypoint(match::MapMatchedPositionConfidenceList const &mapMatchedPositions,
                                       route::FullRoute const &route);

/**
 * @brief find waypoint on the route of the given object, preferable the object center
 *
 * This function first calls findNearestWayPoint on the map matched position
 * confidence list of the center position.
 * If this doesn't lead to a result, the center points of the respective occupied regions are
 * used to find a waypoint. The search starts with the longest region and continues in decreasing order.
 * If this still doesn't lead to a match findNearestWayPoint of the other map matched confidence list positions
 * is taken.
 *
 *  @param[in] object the object which should be searched for within the route
 *  @param[in] route the route to search in
 *
 *  @returns waypoint result containing an iterator pointing to the route segment the lane of
 *   the position is part of and the corresponding route interval having that lane id.
 */
FindWaypointResult findCenterWaypoint(match::Object const &object, route::FullRoute const &route);

/**
 * @brief find nearest waypoint on the route containing the given mapmatched bounding box
 *
 * This function will return the nearest waypoint along the route
 *
 *  @param[in] object which position which should be searched for within the route
 *  @param[in] route the route to search in
 *
 *  @returns waypoint result  containing an iterator pointing to the route segment the lane of
 *   the position is part of and the corresponding route interval having that lane id.
 *   If multiple are matching the one nearest the begin of the route is selected.
 */
FindWaypointResult objectOnRoute(match::MapMatchedObjectBoundingBox const &object, route::FullRoute const &route);

/**
 * @brief find waypoint on the route containing a lane of the given intersection
 *
 * This function will return the nearest waypoint along the route
 *
 *  @param[in] intersection which inner lanes should be searched for within the route
 *  @param[in] route the route to search in
 *
 *  @returns waypoint result  containing an iterator pointing to the route segment the lane of the intersection
 *  is part of and the corresponding route interval having that lane id.
 *  If multiple are matching the one nearest the begin of the route is selected.
 *
 */
FindWaypointResult intersectionOnRoute(intersection::Intersection const &intersection, route::FullRoute const &route);

/**
 * @returns \c true if the connecting route contains road segments which are part of an intersection
 */
bool isConnectedRoutePartOfAnIntersection(route::ConnectingRoute const &connectingRoute);

/** @brief enumeration providing more insights into the actual shorten route result
 */
enum class ShortenRouteResult
{
  /** Shortening succeeded
   */
  Succeeded,
  /** Shortening succeeded: the actual provided position(s) were not found with the
   *   findNearestWaypoint() function, but at least one position is on a lane of the
   *   route start lane segments, just before the route interval starts
   */
  SucceededBeforeRoute,
  /** Shortening succeeded, but the resulting route is empty (i.e. crossed the end of the route)
   */
  SucceededRouteEmpty,
  /** Shortening succeeded: the shortening would cut the current intersection within route.
   *  Therefore, actually not cut.
   */
  SucceededIntersectionNotCut,
  /** Shortening failed, the resulting (or already the input) route is empty
   */
  FailedRouteEmpty
};

/** @brief enumeration defining the mode of operation of the shortenRoute() functions
 */
enum class ShortenRouteMode
{
  /** Standard behavior
   */
  Normal,
  /** If shorten route result would be: ShortenRouteResult::SucceededBeforeRoute, the route is extended
   *  at the beginning so that the result actually becomes ShortenRouteResult::Succeeded
   */
  PrependIfSucceededBeforeRoute,
  /**
   * If the shorten route operation would cut an intersection in between,
   * the route is not shortened further, before the intersection is actually left.
   *
   * If shorten route result would be: ShortenRouteResult::SucceededBeforeRoute, the route is extended
   *  at the beginning so that the result actually becomes ShortenRouteResult::Succeeded
   */
  DontCutIntersectionAndPrependIfSucceededBeforeRoute
};

/** @brief follow the route up to a certain position and shorten the route accordingly
 *
 * @param[in] currentPosition the position which should mark the start of the route after shortening
 * @param[in/out] route the route to be shortened
 * @param[in] extendIfSucceededBeforeRoute if this flag is set @c true,
 *
 * @retval ShortenRouteResult::Succeeded, SucceededBeforeRoute::SucceededBeforeRoute if the shortening request was
 * accepted resulting in a valid route.
 * @retval ShortenRouteResult::SucceededRouteEmpty if the shortening request was accepted resulting in an empty route
 * (i.e. end of route just crossed).
 * @retval ShortenRouteResult::FailedRouteEmpty if the shortening failed and resulting route is empty.
 */
ShortenRouteResult shortenRoute(point::ParaPoint const &currentPosition,
                                route::FullRoute &route,
                                ShortenRouteMode const shortenRouteMode = ShortenRouteMode::Normal);

/** @brief follow the route up to a certain position and shorten the route accordingly
 *
 * Overloaded version of shortenRoute() allow the position to be a vector of positions to consider.
 * If multiple of the positions were actually present within the route, the one is selected providing the
 * longest resulting route.
 *
 * @param[in] currentPositions the vector of positions which should mark the start of the route after shortening
 * @param[in/out] route the route to be shortened
 *
 * @retval ShortenRouteResult::Succeeded, SucceededBeforeRoute::SucceededBeforeRoute if the shortening request was
 * accepted resulting in a valid route.
 * @retval ShortenRouteResult::SucceededRouteEmpty if the shortening request was accepted resulting in an empty route
 * (i.e. end of route just crossed).
 * @retval ShortenRouteResult::FailedRouteEmpty if the shortening failed and resulting route is empty.
 */
ShortenRouteResult shortenRoute(point::ParaPointList const &currentPositions,
                                route::FullRoute &route,
                                ShortenRouteMode const shortenRouteMode = ShortenRouteMode::Normal);

/** @brief follow the route up to a certain position and shorten the route accordingly
 *
 * Overloaded version of shortenRoute() allow the position to be a list of mapmatched positions to consider.
 * If multiple of the positions were actually present within the route, the one is selected providing the
 * longest resulting route.
 *
 * @param[in] mapMatchedPositions the map matched positions which should mark the start of the route after shortening
 * @param[in/out] route the route to be shortened
 *
 * @retval ShortenRouteResult::Succeeded, SucceededBeforeRoute::SucceededBeforeRoute if the shortening request was
 * accepted resulting in a valid route.
 * @retval ShortenRouteResult::SucceededRouteEmpty if the shortening request was accepted resulting in an empty route
 * (i.e. end of route just crossed).
 * @retval ShortenRouteResult::FailedRouteEmpty if the shortening failed and resulting route is empty.
 */
ShortenRouteResult shortenRoute(match::MapMatchedPositionConfidenceList const &mapMatchedPositions,
                                route::FullRoute &route,
                                ShortenRouteMode const shortenRouteMode = ShortenRouteMode::Normal);

/**
 * @brief calculate the RouteParaPoint at a given distance to a given RouteParaPoint
 *
 * @param[in] route
 * @param[in] origin the reference point to this the distance should be calculated
 * @param[in] distance: Negative value is towards begin of the route. Positive values is towards end of the route
 * @param[out] resultingPoint: the point at distance to the origin RouteParaPoint
 *
 * returns @c true if the resultingParaPoint was found in the route
 */
bool calculateRouteParaPointAtDistance(route::FullRoute const &route,
                                       route::RouteParaPoint const &origin,
                                       physics::Distance const &distance,
                                       route::RouteParaPoint &resultingPoint);

/**
 * @brief calculate the RouteParaPoint for a given ParaPoint and FullRoute
 *
 * @param[in] paraPoint is the point under consideration
 * @param[in] route
 * @param[out] routeParaPoint: the converted result
 *
 * returns @c true if the paraPoint was found in the route
 */
bool getRouteParaPointFromParaPoint(point::ParaPoint const &paraPoint,
                                    FullRoute const &route,
                                    route::RouteParaPoint &routeParaPoint);

/**
 * @brief the mode how the route section will be created
 */
enum class RouteSectionCreationMode
{
  SingleLane,   /**< In the resulting route section the LaneSegments contain on single lane interval */
  AllRouteLanes /**< In the resulting route section the LaneSegments contain all the lanes of the original route */
};

/**
 * @brief Return a section of the route that is within the given distance around the given LanePoint
 *
 * This section will contain the LaneInterval that the center point is located at and all
 * the predecessors & successors of this lane up to the given distances or the start / end of the route
 * The length of the resulting route will be distanceFront + distanceEnd except the start / end of the route is within
 * the delta region.
 *
 * @param[in] currentLane specifying the origin of the delta section as FindWaypointResult
 * @param[in] distance to be included in the delta region towards begin of the route
 * @param[in] distance to be included in the delta region towards end of the route
 * @param[in] the route
 * @param[in] routeSectionCreationMode the creation mode for the route section.
 *
 * return the extracted route (will be empty if the centerPoint is not found in the route)
 *
 * @throws std::runtime_error if the route is inconsistent
 */
FullRoute getRouteSection(FindWaypointResult const &currentLane,
                          physics::Distance const &distanceFront,
                          physics::Distance const &distanceEnd,
                          FullRoute const &route,
                          RouteSectionCreationMode const routeSectionCreationMode
                          = RouteSectionCreationMode::SingleLane);

/**
 * @brief Return a section of the route that is within the given distance around the given LanePoint
 *
 * This section will contain the LaneInterval that the center point is located at and all
 * the predecessors & successors of this lane up to the given distances or the start / end of the route
 * The length of the resulting route will be distanceFront + distanceEnd except the start / end of the route is within
 * the delta region.
 *
 * @param[in] centerPoint specifying the origin of the delta section
 * @param[in] distance to be included in the delta region towards begin of the route
 * @param[in] distance to be included in the delta region towards end of the route
 * @param[in] the route
 * @param[in] routeSectionCreationMode the creation mode for the route section.
 *
 * return the extracted route (will be empty if the centerPoint is not found in the route)
 *
 * @throws std::runtime_error if the route is inconsistent
 */
route::FullRoute getRouteSection(point::ParaPoint const &centerPoint,
                                 physics::Distance const &distanceFront,
                                 physics::Distance const &distanceEnd,
                                 route::FullRoute const &route,
                                 RouteSectionCreationMode const routeSectionCreationMode
                                 = RouteSectionCreationMode::SingleLane);

/**
 * @brief Return a section of the route that is around the given object
 *
 * This section will contain the LaneInterval that the center point of the object is located at and all
 * the predecessors & successors of this lane up to half of the objects length or the start / end of the route
 * The length of the resulting route will be object's length except the start / end of the route is within
 * the delta region.
 *
 * @param[in] object specifying the object as the origin and dimension of the delta section
 * @param[in] the route
 * @param[in] routeSectionCreationMode the creation mode for the route section.
 *
 * return the extracted route (will be empty if the object is not found in the route)
 *
 * @throws std::runtime_error if the route is inconsistent
 */
route::FullRoute getRouteSection(match::Object const &object,
                                 route::FullRoute const &route,
                                 RouteSectionCreationMode const routeSectionCreationMode
                                 = RouteSectionCreationMode::SingleLane);

/**
 * @brief struct defining the result type of findFirstLaneChange()
 */
struct FindLaneChangeResult
{
  /**
   * @brief constructor
   *
   * constructs a result that is invalid
   */
  explicit FindLaneChangeResult(FullRoute const &route);

  /** @brief reference to the route */
  FullRoute const &queryRoute;

  /** @brief iterator referencing to route index where the lane change can start at the earliest */
  RoadSegmentList::const_iterator laneChangeStartRouteIterator;

  /** @brief iterator referencing an element of the laneChangeStartRouteIterator content; only valid if
   * laneChangeStartRouteIterator !=
   * queryRoute.routeSegments.end() */
  LaneSegmentList::const_iterator laneChangeStartLaneSegmentIterator;

  /** @brief iterator referencing to route index where the lane change needs to be ended at the latest */
  RoadSegmentList::const_iterator laneChangeEndRouteIterator;

  /** @brief iterator referencing an element of the laneChangeEndRouteIterator content; only valid if
   * laneChangeEndRouteIterator !=
   * queryRoute.routeSegments.end() */
  LaneSegmentList::const_iterator laneChangeEndLaneSegmentIterator;

  /** @brief the lane change direction */
  LaneChangeDirection laneChangeDirection;

  /** @brief the number of connected lane changes to reach the desired target lane from the current position */
  uint32_t numberOfConnectedLaneChanges{0u};

  /** @brief returns true, if the routeIteratorLaneChangeStart and the routeIteratorLaneChangeEnd, as well as the lane
   * change direction are set */
  bool isValid() const
  {
    return laneChangeStartRouteIterator != std::end(queryRoute.roadSegments)
      && laneChangeStartLaneSegmentIterator != std::end(laneChangeStartRouteIterator->drivableLaneSegments)
      && laneChangeEndRouteIterator != std::end(queryRoute.roadSegments)
      && laneChangeEndLaneSegmentIterator != std::end(laneChangeEndRouteIterator->drivableLaneSegments)
      && laneChangeDirection != LaneChangeDirection::Invalid;
  }

  /**
   * @brief calculate the length of the zone of the lane change
   *
   * @returns the length of the lane change zone on the route including start and end lane change segment
   */
  physics::Distance calcZoneLength() const;
};

/** @brief finds the first route interval on a given route where a lane change is necessary to travel the given
 * route from start to end. The function finds the first lane change only.
 *
 * The result is returned by an FindLaneChangeResult which holds
 * - the routeIteratorLaneChangeStart as the pointer to the route index on where the lane change can start at the
 *   earliest. A valid routeIteratorLaneChangeStart can be in the range [std::begin(route), std::end(route)[.
 * - the routeIteratorLaneChangeEnd as the point on the route where the lane change needs to be finished at the
 *   latest. A valid routeIteratorLaneChangeEnd can be in the range [std::begin(route), std::end(route)[
 * - the laneChangeDirection specifying if the lane change goes from left to right or right to left.
 *
 * This function uses the logger of the AdMapAccess::instance() singleton.
 *
 * @param[in] currentPositionEgoVehicle the currentPosition of the ego vehicle
 * @param[in] route the route on which the first lane change is searched
 *
 * @returns a valid lane change result as described above if the route contains a lane change. Returns an invalid
 * FindLaneChangeResult if there is no lane change on the route.
 *
 * @throws std::runtime_error if the route contains invalid transitions.
 * @throws std::runtime_error if the map is not initialized or the road network could not be obtained
 *
 */
FindLaneChangeResult findFirstLaneChange(match::MapMatchedPosition const &currentPositionEgoVehicle,
                                         route::FullRoute const &route);

/** @brief shorten the route from the end to have at maximum the given length
 *
 * A cut of the route is not performed within an intersection, the intersection is kept fully within the route.
 *
 * @param[in/out] route the route to check and to shorten at the end
 * @param[in] length the maximum length
 */
void shortenRouteToDistance(route::FullRoute &route, const physics::Distance &length);

/** @brief function to append a new lane interval to a road segment list
 *
 * In contrast to appendRoadSegmentToRoute() here only the lane interval without any neighbors is added
 *
 * @param[in] laneInterval the new lane interval to be append
 * @param[in] route the route the interval has to be appended
 * @param[in] segmentCountFromDestination the segment count for the new road segment to be created
 *
 */
void appendLaneSegmentToRoute(route::LaneInterval const &laneInterval,
                              route::FullRoute &route,
                              route::SegmentCounter const segmentCountFromDestination = 0u);

/**
 * @brief extends route to have at least the given length
 *
 * @param[in/out] route the route to check and to extend
 * @param[in] length the minimum length
 * @param[in/out] additionalRoutes additional routes in case of intersections on the extension
 *
 * @returns pair of bool and vector
 *
 *  @c true if the route has sufficient length or was extended.
 *          If @c false is returned the route is/was empty/degenerated or the additional routes were not empty.

 */
bool extendRouteToDistance(route::FullRoute &route,
                           physics::Distance const &length,
                           route::FullRouteList &additionalRoutes);

/**
 * @brief extends route with the given list of destinations
 *
 * @param[in/out] route the route to check and to extend
 * @param[in] dest Vector with supporting points as routing parametric points to be visited on the route. Last point in
 * the list is the
 * actual destination point.
 *
 * @returns @c false is returned if the route is/was empty/degenerated
 */
bool extendRouteToDestinations(route::FullRoute &route, const std::vector<route::planning::RoutingParaPoint> &dest);

/**
 * @brief extends route with the given list of destinations
 *
 * @param[in/out] route the route to check and to extend
 * @param[in] dest Vector with supporting points as geo points to be visited on the route. Last point in the list is the
 * actual destination point.
 *
 * @returns @c false is returned if the route is/was empty/degenerated
 */
bool extendRouteToDestinations(route::FullRoute &route, const std::vector<point::GeoPoint> &dest);

/**
 * @brief extends route with the given list of destinations
 *
 * @param[in/out] route the route to check and to extend
 * @param[in] dest Vector with supporting points as ENU points to be visited on the route. Last point in the list is the
 * actual destination point.
 *
 * @returns @c false is returned if the route is/was empty/degenerated
 */
bool extendRouteToDestinations(route::FullRoute &route, const std::vector<point::ENUPoint> &dest);

/** @brief function to append a new lane interval to a road segment list
 *
 * This function is mainly used while creation of a FullRoute object.
 * The lane interval is expanded by it's left and right neighbor lanes (with same driving direction) to a complete
 * road segment; which is then appended to the provided road segment list.
 *
 * After the route is finished, planning::updateRoutePlanningCounters() should be called to update the planning
 * counters.
 *
 * @param[in] laneInterval the new lane interval to be append
 * @param[in] laneOffset the lane offset of the new lane interval to be append
 * @param[in] route the route the interval has to be appended
 *
 */
void appendRoadSegmentToRoute(route::LaneInterval const &laneInverval,
                              route::RouteLaneOffset const &laneOffset,
                              route::FullRoute &route);

/**
 * @brief add an opposing lane segment to an existing (and not empty) road segment with at most the given length
 *
 * This function must only be used, if an existing route should be extended with an opposing lane.
 * Note, the length of the lane segment can be less than the provided distance, in case the neighboring lane segment
 * is shorter as well.
 *
 * The function is used by addOpposingLaneToRoute
 *
 * @param[in] startPoint is the start point where the opposing lane segment will start
 * @param[in] distance is the maximum distance of the newly added lane segment
 * @param[in/out] roadSegment is the road segment which is updated with the opposing lane segment
 * @param[in/out] route is the route the road segment belons to
 *
 * @return -1 on failure, otherwise the length of the new lane segment is added
 */
physics::Distance addOpposingLaneSegmentToRoadSegment(point::ParaPoint const &startpoint,
                                                      physics::Distance const &distance,
                                                      route::RoadSegment &roadSegment,
                                                      route::FullRoute &route);

/**
 * @brief add a part of the opposing lane to an existing route. The part is at most distanceOnWrongLane long,
 *        but may be shorter (e.g. intersections stop adding more segments of the opposing lane)
 *
 * Note the starting point of adding the opposing lane must be in the first segment of the route
 *
 * @param[in] pointOnOppositeLane is the starting point at which the opposite lane should be added
 * @param[in] distanceOnWrongLane maximum distance on the opposite lane
 * @param[in/out] route to be updated
 * @param[out] coveredDistance is the actually covered distance of the opposing lane segment(s)
 */
bool addOpposingLaneToRoute(point::ParaPoint const &pointOnOppositeLane,
                            physics::Distance const &distanceOnWrongLane,
                            route::FullRoute &route,
                            physics::Distance &coveredDistance);

/**
 * @returns the input route expanded by all lanes in opposite driving direction
 *
 * Internally recreates the route by calling appendRoadSegmentToRoute() with a new route
 * having RouteCreationMode::AllRoutableLanes
 */
route::FullRoute getRouteExpandedToOppositeLanes(route::FullRoute const &route);

/**
 * @returns the input route expanded by all neighbor lanes
 *
 * Internally recreates the route by calling appendRoadSegmentToRoute()  with a new route
 * having RouteCreationMode::AllAllNeighbors
 */
route::FullRoute getRouteExpandedToAllNeighborLanes(route::FullRoute const &route);

/**
 * @brief calculate a bypassing route for a given (blocked) route
 *
 * This function is designed to by used to get a route around parking vehicles or for overtaking
 * The output route makes use of neighboring lanes and respects left-hand/right-hand traffic flow
 * The calculation fails when reaching an intersection
 *
 * @param[in] route is the input route, which should not be used
 * @param[out] bypassingRoute is the new route, making use of neighboring lanes
 *
 * @return false if no valid bypassing route was found
 */
bool calculateBypassingRoute(::ad::map::route::FullRoute const &route, ::ad::map::route::FullRoute &bypassingRoute);

/** @brief get borders of a road segment. The road segment is cut at a given parametric offset.
 *
 * The order of the points within the LaneGeometries are ordered according to the route direction.
 *
 * @param[in] roadSegment the road segment to extract the borders from
 * @param[in] parametricOffset parametric offset the borders are cut
 */
lane::ECEFBorder getECEFBorderOfRoadSegment(RoadSegment const &roadSegment,
                                            physics::ParametricValue const parametricOffset);

/** @brief get borders of a road segment
 *
 * The order of the points within the LaneGeometries are ordered according to the route direction.
 *
 * @param[in] roadSegment the road segment to extract the borders from
 */
inline lane::ECEFBorder getECEFBorderOfRoadSegment(RoadSegment const &roadSegment)
{
  return getECEFBorderOfRoadSegment(roadSegment, physics::ParametricValue(1.));
}

/** @brief get borders of a road segment. The road segment is cut at a given parametric offset.
 *
 * The order of the points within the LaneGeometries are ordered according to the route direction.
 *
 * @param[in] roadSegment the road segment to extract the borders from
 * @param[in] parametricOffset parametric offset the borders are cut
 */
lane::ENUBorder getENUBorderOfRoadSegment(RoadSegment const &roadSegment,
                                          physics::ParametricValue const parametricOffset);

/** @brief get borders of a road segment
 *
 * The order of the points within the LaneGeometries are ordered according to the route direction.
 *
 * @param[in] roadSegment the road segment to extract the borders from
 */
inline lane::ENUBorder getENUBorderOfRoadSegment(RoadSegment const &roadSegment)
{
  return getENUBorderOfRoadSegment(roadSegment, physics::ParametricValue(1.));
}

/** @brief get borders of a road segment. The road segment is cut at a given parametric offset.
 *
 * The order of the points within the LaneGeometries are ordered according to the route direction.
 *
 * @param[in] roadSegment the road segment to extract the borders from
 * @param[in] parametricOffset parametric offset the borders are cut
 */
lane::GeoBorder getGeoBorderOfRoadSegment(RoadSegment const &roadSegment,
                                          physics::ParametricValue const parametricOffset);

/** @brief get borders of a road segment
 *
 * The order of the points within the LaneGeometries are ordered according to the route direction.
 *
 * @param[in] roadSegment the road segment to extract the borders from
 */
inline lane::GeoBorder getGeoBorderOfRoadSegment(RoadSegment const &roadSegment)
{
  return getGeoBorderOfRoadSegment(roadSegment, physics::ParametricValue(1.));
}

/** @brief get borders of a full route
 *
 * The order of the points within the LaneGeometries are ordered according to the route direction.
 * The borders of successive road segments are collected in the result vector.
 *
 * @param[in] route the route to extract the borders from
 */
lane::ENUBorderList getENUBorderOfRoute(FullRoute const &route);

/** @brief get borders of a full route
 *
 * The order of the points within the LaneGeometries are ordered according to the route direction.
 * The borders of successive road segments are collected in the result vector.
 *
 * @param[in] route the route to extract the borders from
 */
lane::ECEFBorderList getECEFBorderOfRoute(FullRoute const &route);

/** @brief get borders of a full route
 *
 * The order of the points within the LaneGeometries are ordered according to the route direction.
 * The borders of successive road segments are collected in the result vector.
 *
 * @param[in] route the route to extract the borders from
 */
lane::GeoBorderList getGeoBorderOfRoute(FullRoute const &route);

/**
 * @brief get the ENU heading of a route at the location of the object
 *
 * @param object the object on the route
 * @param route the route
 *
 * @throws std::runtime_error if the objects map matched bounding box is not on the route
 */
point::ENUHeading getENUHeadingOfRoute(match::Object const &object, FullRoute const &route);

/**
 * @brief compare the ENU heading of the route at the location of the object with the ENU heading of the object
 *
 * @returns \c true if the difference between object heading and route heading at the object position
 * is not larger than 90°
 * @throws std::runtime_error if the objects map matched bounding box is not on the route
 */
bool isObjectHeadingInRouteDirection(match::Object const &object, FullRoute const &route);

/**
 * @brief Shorten the provided road segment by a certain distance from the begin
 *
 * The intervals of the segment are shortened as described in shortenIntervalFromBegin()
 *
 * @throws std::runtime_error if the roadSegment is invalid
 */
void shortenSegmentFromBegin(RoadSegment &roadSegment, physics::Distance const &distance);

/**
 * @brief Shorten the provided road segment by a certain distance from the end
 *
 * The intervals of the segment are shortened as described in shortenIntervalFromEnd()
 *
 * @throws std::runtime_error if the roadSegment is invalid
 */
void shortenSegmentFromEnd(RoadSegment &roadSegment, physics::Distance const &distance);

} // namespace route
} // namespace map
} // namespace ad
