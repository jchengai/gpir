// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ad/map/intersection/IntersectionType.hpp"
#include "ad/map/intersection/TurnDirection.hpp"
#include "ad/map/landmark/Types.hpp"
#include "ad/map/lane/Types.hpp"
#include "ad/map/match/Types.hpp"
#include "ad/map/point/Types.hpp"
#include "ad/map/route/RouteOperation.hpp"

/** @brief namespace ad */
namespace ad {
/** @brief namespace map */
namespace map {
/** @brief namespace intersection */
namespace intersection {

/**
 * @brief forward declaration of Intersection class
 */
class Intersection;

/**
 * @brief typedef for shared_ptr of Intersection class
 */
typedef std::shared_ptr<Intersection> IntersectionPtr;

/**
 * @brief typedef for shared_ptr of const Intersection class
 */
typedef std::shared_ptr<Intersection const> IntersectionConstPtr;

/**
 * @class Intersection
 * @brief Logical representation of an intersection along the route
 *
 * There is no public constructor but the static method @ref
 * getIntersectionForRoadSegment which creates an Intersection for a specific
 * road segment of a route and the static method @ref getIntersectionsForRoute
 * which extracts all intersections along the given route. The returned
 * intersections provide the following information:
 * - type of intersection (more precisely the priority at the given entry) -->
 * @ref intersectionType
 * - all lanes inside the intersections (all lanes of type INTERSECTION that are
 * connected to each other)
 *   --> @ref internalLanes
 * - all lanes that directly connect with the intersection --> @ref
 * incomingLanes
 * - all lanes that cross any lane on the route (inside the intersection) -->
 * @ref crossingLanes
 * - all lanes that have higher priority than lanes on route --> @ref
 * lanesWithHigherPriority
 * - get the physics::Distance between an object and the _border_ of the
 * intersection --> @ref physics::DistanceToEntry
 */
class Intersection {
 public:
  /**
   * @brief retrieve the intersection for the given routeIterator
   *
   * @param[in] routeIterator the route iterator of the road segment
   *
   * @return If the route iterator refers to a road segment which is part of an
   * intersection and the preceeding road segment within the route is not part
   * of an intersection, an Intersection object for this specific route through
   * the intersection is created and returned.
   */
  static IntersectionPtr getIntersectionForRoadSegment(
      route::RouteIterator const &routeIterator);

  /**
   * @brief check if the road segment enters an intersection
   *
   * @param[in] routeIterator the route iterator of the road segment
   * @param[out] routePreviousSegmentIter if an intersectin is entered,
   *   this holds the previous route segment that is part of the transition
   *
   * @returns \c true if given routeIterator enters an intersection
   */
  static bool isRoadSegmentEnteringIntersection(
      route::RouteIterator const &routeIterator,
      route::RoadSegmentList::const_iterator &routePreviousSegmentIter);

  /**
   * @brief retrieve all intersections for the given route
   *
   * @param[in] route planned route
   * @return list of all intersections along the route, see
   * getIntersectionForRoadSegment() for details.
   */
  static std::vector<IntersectionPtr> getIntersectionsForRoute(
      route::FullRoute const &route);

  /**
   * @brief retrieve the next intersection for the given route
   *
   * @param[in] route planned route
   *
   * @return If there is an intersection within the route, an Intersection
   * object for the first intersection on the specific route through the
   * intersection is created and returned.
   */
  static IntersectionPtr getNextIntersectionOnRoute(
      route::FullRoute const &route);

  /**
   * @brief check if there is an intersection for the given route
   *
   * @param[in] route planned route
   *
   * @return If there is an intersection within the route, \c true is returned.
   */
  static bool isIntersectionOnRoute(route::FullRoute const &route);

  /**
   * @brief check if a lane is part of any intersection
   */
  static bool isLanePartOfAnIntersection(lane::LaneId const laneId);

  /**
   * @brief check if any lane in the route is part of any intersection
   *
   * @param[in] route planned route
   *
   * @return If there is an intersection within the route, \c true is returned.
   */
  static bool isRoutePartOfAnIntersection(route::FullRoute const &route);

  /** @return the type of this intersection */
  IntersectionType intersectionType() const;

  /** @return the turn direction of the ego route within this intersection */
  TurnDirection turnDirection() const;

  /** @brief return all traffic lights that are relevant for following the route
   */
  landmark::LandmarkIdSet const &applicableTrafficLights() const;

  /** @return all lanes along the route within the intersection */
  lane::LaneIdSet const &lanesOnRoute() const;

  /** @return all parametric points along the route within the intersection */
  point::ParaPointList const &paraPointsOnRoute() const;

  /** @return all lanes that enter intersection on the route */
  lane::LaneIdSet const &incomingLanesOnRoute() const;

  /** @return all lanes that leave the intersection on the route */
  lane::LaneIdSet const &outgoingLanesOnRoute() const;

  /** @return all lanes that enter intersection on the route */
  point::ParaPointList const &incomingParaPointsOnRoute() const;

  /** @return the border points of all lanes that exit the intersection on the
   * route */
  point::ParaPointList const &outgoingParaPointsOnRoute() const;

  /** @return the border points of all lanes that exit the intersection */
  point::ParaPointList const &outgoingParaPoints() const;

  /** @return all lanes that are inside the intersection (independent of route)
   */
  lane::LaneIdSet const &internalLanes() const;

  /** @return all lanes that are inside the intersection and have higher
   * priority (subset of internalLanes) */
  lane::LaneIdSet const &internalLanesWithHigherPriority() const;

  /** @return all lanes that are inside the intersection and have lower priority
   * (subset of internalLanes) */
  lane::LaneIdSet const &internalLanesWithLowerPriority() const;

  /** @return all lanes that lead into the intersection (except the lanes on the
   * route leading into the intersection) */
  lane::LaneIdSet const &incomingLanes() const;

  /** @return all lanes that lead out from the intersection */
  lane::LaneIdSet const &outgoingLanes() const;

  /** @return all lanes that lead into the intersection */
  lane::LaneIdSet const &entryLanes() const;

  /** @return all lanes that lead into the intersection with lower priority
   * (except the lanes on the route leading into the intersection) */
  lane::LaneIdSet const &incomingLanesWithLowerPriority() const;

  /** @return all lanes that lead into the intersection with higher priority
   * (except the lanes on the route leading into the intersection) */
  lane::LaneIdSet const &incomingLanesWithHigherPriority() const;

  /** @return the border points of all lanes that lead into the intersection
   * (incomingLanes() as ParaPoint's) */
  point::ParaPointList const &incomingParaPoints() const;

  /** @return the border points of all lanes that lead into the intersection as
   * ParaPoint's */
  point::ParaPointList const &entryParaPoints() const;

  /** @return the border points of all lanes that lead into the intersection and
   * have higher priority */
  point::ParaPointList const &incomingParaPointsWithHigherPriority() const;

  /** @return the border points of all lanes that lead into the intersection and
   * have lower priority */
  point::ParaPointList const &incomingParaPointsWithLowerPriority() const;

  /** @return all lanes that cross the lanes of the route (in this intersection)
   */
  lane::LaneIdSet const &crossingLanes() const;

  /** @returns the route planning counter of the route used to create this
   * intersection */
  route::RoutePlanningCounter getRoutePlanningCounter() const;

  /** @returns the route segment counter of the road segment this intersection
   * was created */
  route::SegmentCounter getRouteSegmentCountFromDestination() const;

  /** @brief update the route counters according to new route */
  void updateRouteCounters(route::RoutePlanningCounter newRoutePlanningCounter,
                           route::SegmentCounter newRouteSegmentCounter);

  /**
   * @returns the route para point this intersection start
   *
   * That point marks the start of the intersection on the route.
   */
  route::RouteParaPoint getIntersectionStartOnRoute() const;

  /** @returns \c true if the object is within the intersection (touches one of
   * the internalLanes) */
  bool objectWithinIntersection(
      match::MapMatchedObjectBoundingBox const &object) const;

  /** @brief calculate and return the physics::Distance the object
   * interpenetrates with the intersection
   *
   * If the object is not within the intersection: result == 0.
   * If the object is partly within the intersection: 0. < result < object
   * length If the object is fully within the intersection: result == object
   * length If the object is touching incoming/outgoing lanes of different
   * intersection arms at the same time: result == object length
   */
  physics::Distance objectInterpenetrationDistanceWithIntersection(
      match::Object const &object) const;

  /** @brief calculate and return the physics::Distance of the object to the
   * intersection */
  physics::Distance objectDistanceToIntersection(
      match::Object const &object) const;

  /**
   * @returns \c true if the object touches one of the incoming lanes
   */
  bool objectOnIncomingLane(
      match::MapMatchedObjectBoundingBox const &object) const;

  /**
   * @returns \c true if the object touches one of the incoming lanes with
   * higher priority
   */
  bool objectOnIncomingLaneWithHigherPriority(
      match::MapMatchedObjectBoundingBox const &object) const;

  /**
   * @returns \c true if the object touches one of the incoming lanes with lower
   * priority
   */
  bool objectOnIncomingLaneWithLowerPriority(
      match::MapMatchedObjectBoundingBox const &object) const;

  /**
   * @returns \c true if the object touches one of the internal lanes with
   * higher priority
   */
  bool objectOnInternalLaneWithHigherPriority(
      match::MapMatchedObjectBoundingBox const &object) const;

  /**
   * @returns \c true if the object touches one of the internal lanes with lower
   * priority
   */
  bool objectOnInternalLaneWithLowerPriority(
      match::MapMatchedObjectBoundingBox const &object) const;

  /**
   * @returns \c true if the object touches one of the incoming or internal
   * lanes with higher priority
   */
  bool objectOnLaneWithHigherPriority(
      match::MapMatchedObjectBoundingBox const &object) const;

  /**
   * @returns \c true if the object touches one of the incoming or internal
   * lanes with lower priority
   */
  bool objectOnLaneWithLowerPriority(
      match::MapMatchedObjectBoundingBox const &object) const;

  /**
   * @returns \c true if the object touches one of the crossing lanes
   */
  bool objectOnCrossingLane(
      match::MapMatchedObjectBoundingBox const &object) const;

  /**
   * @returns \c true if the object touches the route of the intersection
   */
  bool objectOnIntersectionRoute(
      match::MapMatchedObjectBoundingBox const &object) const;

  /**
   * @returns \c true if the provided \c objectRoute crosses the route of the
   * intersection
   *
   * This is the case if one of the crossingLanes() is part of the objectRoute.
   */
  bool objectRouteCrossesIntersectionRoute(
      route::FullRoute const &objectRoute) const;

  /**
   * @returns \c true if the beginning of the provided \c objectRoute is coming
   * from the same arm as the intersection route
   *
   * This means the object is driving from same direction through the
   * intersection.
   */
  bool objectRouteFromSameArmAsIntersectionRoute(
      route::FullRoute const &objectRoute) const;

  /**
   * @returns \c true if the beginning of the provided \c objectRoute is coming
   * from the same arm where the intersection route exits and leaves the
   * intersection from the same arm the intersection route enters.
   *
   * This means the object is driving from exactly the opposite direction
   * through the intersection.
   */
  bool objectRouteOppositeToIntersectionRoute(
      route::FullRoute const &objectRoute) const;

  /**
   * @returns \c true if the provided \c objectRoute crosses a internal lane
   * with higher priority
   *
   * This is the case if one of the internalLanesWithHigherPriority()
   * is part of the objectRoute.
   */
  bool objectRouteCrossesLanesWithHigherPriority(
      route::FullRoute const &objectRoute) const;

  /**
   * @returns \c true if the provided \c objectRoute contains an internal lane
   *
   * This is the case if one of the internalLanes()
   * is part of the objectRoute.
   */
  bool objectRouteCrossesIntersection(
      route::FullRoute const &objectRoute) const;

  /**
   * @brief return the speed limit of this intersection
   */
  physics::Speed getSpeedLimit() const;

  /**
   * @brief checks if there is just solid traffic lights on the intersection
   * entry.
   *
   * @return true just if solid traffic lights are found
   */
  bool onlySolidTrafficLightsOnRoute();

 private:
  /**
   * @brief checks if is an solid traffic light.
   *
   * @param[in] trafficLightId TrafficLightId.
   * @return true if is an solid traffic light
   */
  bool isSolidTrafficLight(landmark::LandmarkId trafficLightId);

  /**
   * @brief Extract traffic light type for a specific traffic light id .
   *
   * @param[in] trafficLightId TrafficLightId.
   * @return traffic light type.
   */
  landmark::TrafficLightType extractTrafficLightType(
      landmark::LandmarkId trafficLightId);

  /**
   * @brief consolidate all required information about an intersection
   *
   * @param route the full route that was passed to @ref
   * getIntersectionsForRoute
   * @param firstSegmentWithinIntersection the route segment iterator pointing
   * the first segment when entering the intersection. The route that is taken
   * through the intersection defines the preference lanes depending on the
   * right of way at the given entry lane
   *
   * Collect all lanes that are part of the intersection and touch the
   * intersection (in terms of lanes leading into the intersection as well as
   * lanes leading out of the intersection).
   *
   * Iterates over all contact lanes (and the contact lanes of those lanes) to
   * collect all lanes of type INTERSECTION which have a direct (or indirect)
   * contact with the given lane.
   *
   * The information is provided through four different sets of lanes
   * 1. All lanes that are inside the intersection (independent of the route)
   *    --> internalLanes()
   * 2. All lanes that are before the intersection up to a given
   * physics::Distance
   *    --> incomingLanes()
   *    (this excludes the lanes of the entry where the route is entering the
   * intersection)
   * 3. All lanes that have a higher priority than the entering lane (of the
   * route)
   *    --> lanesWithHigherPriority()
   * 4. All lanes that cross/intersection with the lanes on the route,
   * independent of the traffic regulation
   *    --> crossingLanes()
   */
  Intersection(route::FullRoute const &route,
               route::RoadSegmentList::const_iterator const
                   &lastSegmentBeforeIntersection,
               route::RoadSegmentList::const_iterator const
                   &firstSegmentWithinIntersection);
  Intersection() = delete;

  IntersectionType mIntersectionType{IntersectionType::Unknown};

  //! the route planning counter of the route used to create this intersection
  route::RoutePlanningCounter mRoutePlanningCounter;

  //! the route segment counter of the first road segment within the
  //! intersection
  route::SegmentCounter mSegmentCountFromDestination;

  //! all lanes along the route within the intersection
  lane::LaneIdSet mLanesOnRoute;

  //! all parametric points along the route within the intersection
  point::ParaPointList mParaPointsOnRoute;

  //! all lanes that enter intersection on the route
  lane::LaneIdSet mIncomingLanesOnRoute;

  //! all border points of lanes that enter intersection on the route
  point::ParaPointList mIncomingParaPointsOnRoute;

  //! all lanes inside the intersection
  lane::LaneIdSet mInternalLanes;

  //! all lanes that have priority over the lanes of the route
  lane::LaneIdSet mInternalLanesWithHigherPriority;

  //! all lanes that not have priority over the lanes of the route
  //! ((mInternalLanes - mLanesOnRoute) - mInternalLanesWithHigherPriority)
  lane::LaneIdSet mInternalLanesWithLowerPriority;

  //! all lanes within the intersection that can be reached from the incoming
  //! intersection arm
  lane::LaneIdSet mInternalLanesFromSameIntersectionArm;

  //! lanes going into the intersection (excluding lanes on path)
  lane::LaneIdSet mIncomingLanes;

  //! lanes going into the intersection
  lane::LaneIdSet mEntryLanes;

  //! incoming lanes (not on the route) represented as ParaPoint
  point::ParaPointList mIncomingParaPoints;

  //! lanes going into the intersection represented as ParaPoint
  point::ParaPointList mEntryParaPoints;

  //! lanes going out of the intersection
  lane::LaneIdSet mOutgoingLanes{};

  //! all lanes that exit the intersection on the route
  lane::LaneIdSet mOutgoingLanesOnRoute{};

  //! all border points of lanes that exits intersection on the route
  point::ParaPointList mOutgoingParaPointsOnRoute;

  //! all incoming lanes that have higher priority over the incoming lanes of
  //! the route represented as ParaPoint
  point::ParaPointList mIncomingParaPointsWithHigherPriority;

  //! all incoming lanes that have higher priority over the incoming lanes of
  //! the route
  lane::LaneIdSet mIncomingLanesWithHigherPriority;

  //! all incoming lanes that have lower priority over the incoming lanes of the
  //! route represented as ParaPoint
  point::ParaPointList mIncomingParaPointsWithLowerPriority;

  //! all incoming lanes that have lower priority over the incoming lanes of the
  //! route
  lane::LaneIdSet mIncomingLanesWithLowerPriority;

  //! all lanes that cross any lane of the route inside the intersection
  lane::LaneIdSet mCrossingLanes;

  //! outgoing lanes represented as ParaPoint
  point::ParaPointList mOutgoingParaPoints;

  //! intersection arms order and lanes for each arm
  std::map<TurnDirection, lane::LaneIdSet> mIntersectionArms;

  landmark::LandmarkIdSet mTrafficLightIds;

  TurnDirection mTurnDirection{TurnDirection::Unknown};

  //! the speed limit of this intersection
  physics::Speed mSpeedLimit;

  /**
   * Managing relations between lanes through separate maps with sets. Reading:
   * key: id of lane
   * value: set of lanes that relate with this one (e.g. overlap)
   */
  std::map<lane::LaneId, lane::LaneIdSet> mOverlapping;
  std::map<lane::LaneId, lane::LaneIdSet> mSuccessor;
  std::map<lane::LaneId, lane::LaneIdSet> mPredecessor;

  void extractRightOfWayAndCollectTrafficLights(
      route::LaneInterval const &laneInterval,
      lane::LaneIdSet const &successors, lane::LaneId &toLaneId);

  void extractLanesOfIntersection(lane::LaneId const laneId);

  void extractCrossingLanes();
  void extractLanesWithHigherPriority();
  void extractLanesWithLowerPriority();
  void extractLanesFromSameIntersectionArm();
  void calculateParaPoints();

  /**
   * @brief Entry Parapoints with higher priority.
   *
   * this functions finds for each intersection lane entry if one of is
   * successors is a internal lane with priority, taking in account ego route
   * through the intersection.
   */
  void calculateEnteringProrityParaPoints();
  void insertIncomingLane(lane::LaneId const laneId);
  void insertOutgoingLane(lane::LaneId const laneId);

  /**
   * @brief Group lanes outside of the intersection by arm.
   *
   * The basic idea for dealing with the priority rules at an intersection are
   * to group all incoming and outgoing lanes by arms and to order these arms
   * counterclockwise starting at the first arm on the right just after the arm
   * where the route is entering the intersection. The lanes of the arm where
   * the route is entering the intersection are not ordered and stored.
   *
   * Furthermore, based on the extracted angles, the turn direction extracted
   * that is attached to this intersection (left, straight, right).
   *
   *           1
   *          | |
   *          e f
   *          | |
   *  -- g --    -- d --
   * 2                  0
   *  -- h --    -- c --
   *          | |
   *          a b
   *          | |
   *          ego
   * In the example above the route enters the intersection at arm ego. All
   * lanes (a,b) are not assigned, whereas lanes (c,d) are assigned to the arm
   * with index 0, and so on. This ordering is later used in
   * @ref extractLanesWithHigherPriority and its concrete handling of the
   * different priority rules to derive the lanes with higher priority.
   */
  void orderIntersectionArmsAndExtractTurnDirection();

  //! check if given lane is inside the intersection
  bool laneIsPartOfIntersection(lane::LaneId const laneId) const;
  void processContactsForLane(lane::Lane const &lane,
                              lane::ContactLane const &contact);

  /**
   * @brief Derive lanes with higher priority in case the host vehicle has right
   * of way.
   *
   * @note turning priority (German: abknickende Vorfahrt) is not handled
   * correctly.
   *           1
   *          | |
   *          e f
   *          | |
   *  -- g --    -- d --
   * 2                  0
   *  -- h --    -- c --
   *          | |
   *          a b(has way)
   *          | |
   *          ego
   * Logic:
   * If a lane crosses (a lane on) the route and that lane has right of way it
   * is considered to be a lane with higher priority. If that lane does not have
   * right of way it is not considered.
   *
   * Example:
   * In the example above the route enters the intersection at lane b and exits
   * on lane g. in this situation the route enters the intersection on a
   * priority lane and then turns to left, in this situation it is necessary
   * give priority to the traffic that comes from the lane e arm 1.
   */
  void adjustLanesForHasWay();

  /**
   * @brief Derive lanes with higher priority in case the host vehicle has to
   * yield.
   *
   * @note turning priority (German: abknickende Vorfahrt) is not handled
   * correctly.
   *           1
   *          | |
   *          e f
   *          | |
   *  -- g --    -- d --
   * 2                  0
   *  -- h --    -- c --
   *          | |
   *          a b(yield)
   *          | |
   *          ego
   * Example:
   * In the example above the route enters the intersection at lane b and exits
   * on lane g. In this situation it is need to give priority to all lanes that
   * intersect with the route inside of the intersection.
   */
  void adjustLanesForYield();

  /**
   * @brief Derive lanes with higher priority in case the host vehicle has to
   * give priority to the right.
   *
   * When the arm where the route is entering the intersection due the traffic
   * regulation need to apply the rule priority to the right, this function
   * provide the lanes to give priority inside the intersection.
   *           1
   *          | |
   *          e f
   *          | |
   *  -- g --    -- d --
   * 2                  0
   *  -- h --    -- c --
   *          | |
   *          a b
   *          | |
   *          ego
   * Logic:
   * If the arm is before the arm where the route exits the intersection
   * (counterclockwise direction) for all successor lanes that are inside of the
   * intersection for that arm, it is necessary to give priority.
   *
   * Example:
   * In the example above the route enters the intersection at lane b and exits
   * on lane g. In this situation it is needed to give priority to all lanes
   * that intersect with the route inside of the intersection for traffic coming
   * from lanes d and e. If the route is from lane b to lane f, it will be
   * necessary to give priority to the traffic coming from lane d only.
   */
  void adjustPriorityToRight();

  /**
   * @brief Provide lanes for traffic lights (in case the traffic light(s)
   * is/are green)
   *
   * Not clear yet how to exactly deal with the state of the different traffic
   * lights.
   */
  void adjustLanesForTrafficLight();

  /**
   * @brief Returns \c true if the turn direction is opposing the traffic
   * direction
   *
   * On right handed traffic this is true if the route turns to the left,
   * on left handed traffic if the route turns to the right.
   */
  bool turnDirectionCrossesStraightTraffic() const;

  /** @brief gather all traffic lights that apply when going from fromLaneId to
   * toLaneId */
  void collectTrafficLights(lane::LaneId fromLaneId, lane::LaneId toLaneId,
                            bool useSuccessor);

  lane::LaneIdSet successorsOnRouteLeavingIntersection(
      route::FullRoute const &route,
      route::RoadSegmentList::const_iterator const &roadSegmentIt,
      route::LaneSegment const &laneSegment);

  bool segmentLeavesIntersectionOnRoute(
      route::FullRoute const &route,
      route::RoadSegmentList::const_iterator const &roadSegmentIt,
      lane::LaneId laneId);

  lane::LaneIdSet successorsOnRouteLeavingIntersection(
      route::LaneSegment const &laneSegment);

  void calculateSpeedLimit();

  enum SuccessorMode { OwnIntersection, AnyIntersection };

  bool isLanePartOfIntersection(lane::LaneId const laneId,
                                SuccessorMode const successorMode) const;

  /**
   * @brief Provide the direct successor lane segments in lane direction within
   * and outside of the intersection.
   *
   * @param laneId LaneId
   * @return a pair with <the laneID segments within the intersection, the
   * laneID segments outside the intersection>
   */
  std::pair<lane::LaneIdSet, lane::LaneIdSet>
  getDirectSuccessorsInLaneDirection(lane::LaneId const laneId,
                                     SuccessorMode const successorMode) const;

  /**
   * @brief Provide the direct successor lane segments in lane direction within
   * the intersection.
   *
   * @param laneId LaneId
   * @return the laneID segments within the intersection.
   */
  lane::LaneIdSet getDirectSuccessorsInLaneDirectionWithinIntersection(
      lane::LaneId const laneId, SuccessorMode const successorMode) const;

  /**
   * @brief Provide the successor lane segments in lane direction within the
   * intersection recursively until the intersection is left
   *
   * @param laneId LaneId
   * @return the laneID segments within the intersection.
   */
  lane::LaneIdSet getAllSuccessorsInLaneDirectionWithinIntersection(
      lane::LaneId const laneId, SuccessorMode const successorMode) const;

  /**
   * @brief Provide the successor lane segments in lane direction within the
   * intersection recursively until the intersection is left including the input
   * laneId if part of the inner lanes.
   *
   * @param laneId LaneId
   * @return the laneID segments within the intersection.
   */
  lane::LaneIdSet getLaneAndAllSuccessorsInLaneDirectionWithinIntersection(
      lane::LaneId const laneId, SuccessorMode const successorMode) const;

  /**
   * @brief Provide the outgoing lane segments that are reachable in lane
   * direction
   *
   * @param laneId LaneId
   * @return the outgoing laneID segments outside the intersection.
   */
  lane::LaneIdSet getAllReachableOutgoingLanes(
      lane::LaneId const laneId, SuccessorMode const successorMode) const;

  /**
   * @brief Provide the outgoing lane segments that are reachable in lane
   * direction as well as the intersection internal lanes
   *
   * @param laneId LaneId
   * @return a pair with <the laneID segments within the intersection, the
   * laneID segments outside the intersection>
   */
  std::pair<lane::LaneIdSet, lane::LaneIdSet>
  getAllReachableInternalAndOutgoingLanes(
      lane::LaneId const laneId, SuccessorMode const successorMode) const;

  /**
   * @brief Add the lane and all successor of it within the intersection to the
   * list of lanes with higher priority.
   *
   * @param laneId LaneId
   * @param restrictToOutgoingIntersectionArm if not set to Unknown, the lanes
   * to be added are restricted in addition to lanes which leave the
   * intersection at the respective intersection arm
   */
  void addLaneAndSuccessorsToInternalLanesWithHigherPriority(
      lane::LaneId const &lanes,
      TurnDirection const restrictToOutgoingIntersectionArm =
          TurnDirection::Unknown);

  /**
   * @brief Add the lanes and all successor of it within the intersection to the
   * list of lanes with higher priority.
   *
   * @param lanes the lanes
   * @param restrictToOutgoingIntersectionArm if not set to Unknown, the lanes
   * to be added are restricted in addition to lanes which leave the
   * intersection at the respective intersection arm
   */
  void addLaneAndSuccessorsToInternalLanesWithHigherPriority(
      lane::LaneIdSet const &lanes,
      TurnDirection const restrictToOutgoingIntersectionArm =
          TurnDirection::Unknown);

  /**
   * @brief Add the lane and all successor of it within the intersection to the
   * list of lanes with higher priority in case the lane is crossing with the
   * route.
   *
   * @param laneId LaneId
   * @param restrictToOutgoingIntersectionArm if not set to Unknown, the lanes
   * to be added are restricted in addition to lanes which leave the
   * intersection at the respective intersection arm
   */
  void addLaneAndSuccessorsToInternalLanesWithHigherPriorityIfCrossing(
      lane::LaneId const &lanes,
      TurnDirection const restrictToOutgoingIntersectionArm =
          TurnDirection::Unknown);

  /**
   * @brief Add the lanes and all successor of it within the intersection to the
   * list of lanes with higher priority in case the lane is crossing with the
   * route.
   *
   * @param lanes the lanes
   * @param restrictToOutgoingIntersectionArm if not set to Unknown, the lanes
   * to be added are restricted in addition to lanes which leave the
   * intersection at the respective intersection arm
   */
  void addLaneAndSuccessorsToInternalLanesWithHigherPriorityIfCrossing(
      lane::LaneIdSet const &lanes,
      TurnDirection const restrictToOutgoingIntersectionArm =
          TurnDirection::Unknown);

  /**
   * @brief Check if from a lane one is able to leave the intersection at the
   * respective intersection arm
   *
   * @param laneId LaneId of lane to check
   * @param outgoingIntersectionArm the outgoind intersection arm index to
   * consider
   *
   * @returns \c true if the lane belongs to a path that can exit the
   * intersection at the given intersection arm
   */
  bool outgoingIntersectionArmCanBeReached(
      lane::LaneId const laneId, TurnDirection const outgoingIntersectionArm);
};

}  // namespace intersection
}  // namespace map
}  // namespace ad

namespace std {

/**
 * \brief standard ostream operator for ad::map::intersection::Intersection
 *
 * \param[in] os The output stream to write to
 * \param[in] intersection The intersection object
 *
 * \returns The stream object.
 *
 */
static inline std::ostream &operator<<(
    std::ostream &os,
    ::ad::map::intersection::Intersection const &intersection) {
  os << "Intersection[";
  os << toString(intersection.intersectionType());
  os << "]" << std::endl;
  os << "->internalLanes: ";
  os << intersection.internalLanes();
  os << std::endl;
  os << "->internalLanesWithHigherPriority: ";
  os << intersection.internalLanesWithHigherPriority();
  os << std::endl;
  os << "->incomingLanes: ";
  os << intersection.incomingLanes();
  os << std::endl;
  os << "->incomingParaPoints: ";
  os << intersection.incomingParaPoints();
  os << std::endl;
  os << "->incomingParaPointsWithHigherPriority: ";
  os << intersection.incomingParaPointsWithHigherPriority();
  os << std::endl;
  os << "->crossingLanes";
  os << intersection.crossingLanes();
  os << std::endl;
  os << "->lanesOnRoute: ";
  os << intersection.lanesOnRoute();
  os << std::endl;
  os << "->incomingLanesOnRoute: ";
  os << intersection.incomingLanesOnRoute();
  os << std::endl;
  os << "->incomingParaPointsOnRoute: ";
  os << intersection.incomingParaPointsOnRoute();
  return os;
}

/**
 * \brief overload of the std::to_string for ad::map::intersection::Intersection
 */
static inline std::string to_string(
    ::ad::map::intersection::Intersection const &intersection) {
  stringstream sstream;
  sstream << intersection;
  return sstream.str();
}
}  // namespace std
