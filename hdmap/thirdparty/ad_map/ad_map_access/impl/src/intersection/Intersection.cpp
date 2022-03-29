// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/intersection/Intersection.hpp"

#include <math.h>

#include "ad/map/access/Logging.hpp"
#include "ad/map/access/Operation.hpp"
#include "ad/map/landmark/LandmarkOperation.hpp"
#include "ad/map/lane/LaneOperation.hpp"
#include "ad/map/point/Operation.hpp"
#include "ad/map/route/RouteOperation.hpp"

namespace ad {
namespace map {
namespace intersection {

/*********************************************************************
 * common functions not specific for intersections
 *********************************************************************/
bool Intersection::isLanePartOfIntersection(lane::LaneId const laneId, SuccessorMode const successorMode) const
{
  if (successorMode == SuccessorMode::OwnIntersection)
  {
    return mInternalLanes.count(laneId) > 0u;
  }
  else
  {
    return isLanePartOfAnIntersection(laneId);
  }
}

std::pair<lane::LaneIdSet, lane::LaneIdSet>
Intersection::getDirectSuccessorsInLaneDirection(lane::LaneId const laneId, SuccessorMode const successorMode) const
{
  auto lane = lane::getLane(laneId);
  auto location = lane::ContactLocation::SUCCESSOR;
  if (lane.direction == lane::LaneDirection::NEGATIVE)
  {
    location = lane::ContactLocation::PREDECESSOR;
  }
  std::pair<lane::LaneIdSet, lane::LaneIdSet> result;
  for (auto const &contact : lane::getContactLanes(lane, location))
  {
    lane::LaneId const successorLane = contact.toLane;
    if (isLanePartOfIntersection(successorLane, successorMode))
    {
      // restrict first to within intersection
      result.first.insert(successorLane);
    }
    else
    {
      // restrict second to outside intersection
      result.second.insert(successorLane);
    }
  }
  return result;
}

lane::LaneIdSet
Intersection::getDirectSuccessorsInLaneDirectionWithinIntersection(lane::LaneId const laneId,
                                                                   SuccessorMode const successorMode) const
{
  return getDirectSuccessorsInLaneDirection(laneId, successorMode).first;
}

lane::LaneIdSet Intersection::getAllSuccessorsInLaneDirectionWithinIntersection(lane::LaneId const laneId,
                                                                                SuccessorMode const successorMode) const
{
  lane::LaneIdSet result;
  auto directSuccessors = getDirectSuccessorsInLaneDirectionWithinIntersection(laneId, successorMode);
  for (auto directSuccessorId : directSuccessors)
  {
    auto directSuccessorRecursiveResult
      = getAllSuccessorsInLaneDirectionWithinIntersection(directSuccessorId, successorMode);
    result.insert(directSuccessorRecursiveResult.begin(), directSuccessorRecursiveResult.end());
  }
  result.insert(directSuccessors.begin(), directSuccessors.end());
  return result;
}

lane::LaneIdSet
Intersection::getLaneAndAllSuccessorsInLaneDirectionWithinIntersection(lane::LaneId const laneId,
                                                                       SuccessorMode const successorMode) const
{
  auto result = getAllSuccessorsInLaneDirectionWithinIntersection(laneId, successorMode);
  if (isLanePartOfIntersection(laneId, successorMode))
  {
    result.insert(laneId);
  }
  return result;
}

lane::LaneIdSet Intersection::getAllReachableOutgoingLanes(lane::LaneId const laneId,
                                                           SuccessorMode const successorMode) const
{
  lane::LaneIdSet result;
  auto directSuccessors = getDirectSuccessorsInLaneDirection(laneId, successorMode);
  // recursive until end of intersection
  for (auto directSuccessorId : directSuccessors.first)
  {
    auto directSuccessorRecursiveResult = getAllReachableOutgoingLanes(directSuccessorId, successorMode);
    result.insert(directSuccessorRecursiveResult.begin(), directSuccessorRecursiveResult.end());
  }
  result.insert(directSuccessors.second.begin(), directSuccessors.second.end());
  return result;
}

std::pair<lane::LaneIdSet, lane::LaneIdSet>
Intersection::getAllReachableInternalAndOutgoingLanes(lane::LaneId const laneId,
                                                      SuccessorMode const successorMode) const
{
  std::pair<lane::LaneIdSet, lane::LaneIdSet> result;
  auto directSuccessors = getDirectSuccessorsInLaneDirection(laneId, successorMode);
  // recursive until end of intersection
  for (auto directSuccessorId : directSuccessors.first)
  {
    auto directSuccessorRecursiveResult = getAllReachableInternalAndOutgoingLanes(directSuccessorId, successorMode);
    result.first.insert(directSuccessorRecursiveResult.first.begin(), directSuccessorRecursiveResult.first.end());
    result.second.insert(directSuccessorRecursiveResult.second.begin(), directSuccessorRecursiveResult.second.end());
  }
  result.first.insert(directSuccessors.first.begin(), directSuccessors.first.end());
  result.second.insert(directSuccessors.second.begin(), directSuccessors.second.end());
  return result;
}

bool Intersection::isLanePartOfAnIntersection(lane::LaneId const laneId)
{
  auto lane = lane::getLane(laneId);
  return lane::isLanePartOfAnIntersection(lane);
}

bool Intersection::isRoutePartOfAnIntersection(route::FullRoute const &route)
{
  for (auto const &roadSegment : route.roadSegments)
  {
    for (auto const &laneSegment : roadSegment.drivableLaneSegments)
    {
      if (isLanePartOfAnIntersection(laneSegment.laneInterval.laneId))
      {
        return true;
      }
    }
  }
  return false;
}

bool laneEntersIntersection(lane::LaneId const &from, lane::LaneId const &to)
{
  return ((from != to) && !Intersection::isLanePartOfAnIntersection(from)
          && Intersection::isLanePartOfAnIntersection(to));
}

/*********************************************************************
 * Intersection related functions
 *********************************************************************/
IntersectionType fromContactTypes(lane::ContactTypeList const &types)
{
  for (auto contactType : types)
  {
    // fallthrough is intended here
    switch (contactType)
    {
      // clang-format off
      // switched off to keep the formatting of the comments
      case lane::ContactType::FREE:               ///< There is no physical or legal obstacles between two objects.
        return IntersectionType::HasWay;
      case lane::ContactType::STOP:               ///< STOP regulation at the end of the lane.
        return IntersectionType::Stop;
      case lane::ContactType::STOP_ALL:           ///< STOP 3-way, 4-way etc regulation at the end of the lane.
        return IntersectionType::AllWayStop;
      case lane::ContactType::YIELD:              ///< YIELD regulation at the end of the lane.
        return IntersectionType::Yield;
      case lane::ContactType::CROSSWALK:          ///< Crosswalk at the end of the lane.
        return IntersectionType::Crosswalk;
      case lane::ContactType::PRIO_TO_RIGHT:      ///< Priority to right at the end of the lane.
        return IntersectionType::PriorityToRight;
      case lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT:      ///< Priority to right and straight at the end of the lane.
        return IntersectionType::PriorityToRightAndStraight;
      case lane::ContactType::TRAFFIC_LIGHT:      ///< Traffic light on the way
        return IntersectionType::TrafficLight;
      case lane::ContactType::SPEED_BUMP:         ///< Speed bump at the end of the lane.
      case lane::ContactType::INVALID:
      case lane::ContactType::UNKNOWN:            ///< Unknown value.
      case lane::ContactType::LANE_CHANGE:        ///< Transition between one lane to direct lateral neighboor.
      case lane::ContactType::LANE_CONTINUATION:  ///< Transition between one lane to longitudinal direct neighboor.
      case lane::ContactType::LANE_END:           ///< End of the Lane - line.
      case lane::ContactType::SINGLE_POINT:       ///< End of the Lane - point.
      case lane::ContactType::GATE_BARRIER:       ///< Gate with barrier at the end of the lane.
      case lane::ContactType::GATE_TOLBOOTH:      ///< Tolbooth barrier at the end of the lane.
      case lane::ContactType::GATE_SPIKES:        ///< Spikes (in direction) at the end of the lane.
      case lane::ContactType::GATE_SPIKES_CONTRA: ///< Spikes (in opposite direction) at the end of the lane.
      case lane::ContactType::CURB_UP:            ///< Curb up at the object side.
      case lane::ContactType::CURB_DOWN:          ///< Curb down at the object.
      case lane::ContactType::RIGHT_OF_WAY:
        break;      // in all other cases, we do nothing, maybe another contact has valid information.
      // clang-format on
      default:
        throw std::invalid_argument("Intersection fromContactTypes: Invalid contact type passed");
    }
  }
  return IntersectionType::Unknown;
}

IntersectionType getRightOfWayForTransition(lane::LaneId fromLaneId, lane::LaneId toLaneId, bool useSuccessor)
{
  auto fromLane = lane::getLane(fromLaneId);
  auto location = lane::ContactLocation::PREDECESSOR;
  if (useSuccessor)
  {
    location = lane::ContactLocation::SUCCESSOR;
  }
  auto contacts = getContactLanes(fromLane, location);
  for (auto const &contact : contacts)
  {
    if (contact.toLane == toLaneId)
    {
      auto result = fromContactTypes(contact.types);
      if (result != IntersectionType::Unknown)
      {
        return result;
      }
    }
  }
  return IntersectionType::Unknown;
}

point::ParaPoint getIncomingParaPoint(lane::LaneId const laneId)
{
  point::ParaPoint p;
  p.laneId = laneId;
  p.parametricOffset
    = (lane::isLaneDirectionNegative(laneId) ? physics::ParametricValue(0.) : physics::ParametricValue(1.));
  return p;
}

point::ParaPoint getOutgoingParaPoint(lane::LaneId const laneId)
{
  point::ParaPoint p;
  p.laneId = laneId;
  p.parametricOffset
    = (lane::isLaneDirectionNegative(laneId) ? physics::ParametricValue(1.) : physics::ParametricValue(0.));
  return p;
}

bool Intersection::isRoadSegmentEnteringIntersection(route::RouteIterator const &routeIterator,
                                                     route::RoadSegmentList::const_iterator &routePreviousSegmentIter)
{
  if ((routeIterator.roadSegmentIterator != routeIterator.route.roadSegments.end())
      && (routeIterator.roadSegmentIterator != routeIterator.route.roadSegments.begin()))
  {
    auto previousSegmentIter = routeIterator.roadSegmentIterator;
    previousSegmentIter--;
    // only look at the first parapoint per index
    auto const fromLane = previousSegmentIter->drivableLaneSegments.front().laneInterval.laneId;
    auto const toLane = routeIterator.roadSegmentIterator->drivableLaneSegments.front().laneInterval.laneId;
    if (laneEntersIntersection(fromLane, toLane))
    {
      routePreviousSegmentIter = previousSegmentIter;
      return true;
    }
  }
  return false;
}

IntersectionPtr Intersection::getIntersectionForRoadSegment(route::RouteIterator const &routeIterator)
{
  IntersectionPtr result;
  route::RoadSegmentList::const_iterator routePreviousSegmentIter;
  bool const intersectionStart = isRoadSegmentEnteringIntersection(routeIterator, routePreviousSegmentIter);
  if (intersectionStart)
  {
    try
    {
      result = IntersectionPtr(
        new Intersection(routeIterator.route, routePreviousSegmentIter, routeIterator.roadSegmentIterator));
    }
    catch (...)
    {
      access::getLogger()->warn("Failed to create intersection at route iterator {} for route: {}",
                                *routeIterator.roadSegmentIterator,
                                routeIterator.route);
    }
  }
  return result;
}

std::vector<IntersectionPtr> Intersection::getIntersectionsForRoute(route::FullRoute const &route)
{
  std::vector<IntersectionPtr> result;
  for (auto roadSegmentIter = route.roadSegments.begin(); roadSegmentIter != route.roadSegments.end();
       roadSegmentIter++)
  {
    // @todo: also consider segments leaving intersection if first segment is already within intersection
    auto roadSegmentResult = getIntersectionForRoadSegment(route::RouteIterator(route, roadSegmentIter));
    if (bool(roadSegmentResult))
    {
      result.push_back(roadSegmentResult);
    }
  }
  return result;
}

IntersectionPtr Intersection::getNextIntersectionOnRoute(route::FullRoute const &route)
{
  IntersectionPtr result;
  for (auto roadSegmentIter = route.roadSegments.begin(); roadSegmentIter != route.roadSegments.end();
       roadSegmentIter++)
  {
    // @todo: also consider segments leaving intersection if first segment is already within intersection
    result = getIntersectionForRoadSegment(route::RouteIterator(route, roadSegmentIter));
    if (bool(result))
    {
      return result;
    }
  }
  return result;
}

bool Intersection::isIntersectionOnRoute(route::FullRoute const &route)
{
  for (auto roadSegmentIter = route.roadSegments.begin(); roadSegmentIter != route.roadSegments.end();
       roadSegmentIter++)
  {
    route::RoadSegmentList::const_iterator routePreviousSegmentIter;
    // @todo: also consider segments leaving intersection if first segment is already within intersection
    if (isRoadSegmentEnteringIntersection(route::RouteIterator(route, roadSegmentIter), routePreviousSegmentIter))
    {
      return true;
    }
  }
  return false;
}

bool Intersection::segmentLeavesIntersectionOnRoute(route::FullRoute const &route,
                                                    route::RoadSegmentList::const_iterator const &roadSegmentIt,
                                                    lane::LaneId laneId)
{
  // end of iterator? -> false
  // lane is of type intersection? -> iterate through successors
  // lane of type normal -> true
  if (roadSegmentIt == route.roadSegments.end())
  {
    return false;
  }
  if (isLanePartOfAnIntersection(laneId))
  {
    mLanesOnRoute.insert(laneId);
    for (auto const &segment : roadSegmentIt->drivableLaneSegments)
    {
      if (segment.laneInterval.laneId == laneId)
      {
        for (auto successorId : segment.successors)
        {
          if (segmentLeavesIntersectionOnRoute(route, roadSegmentIt + 1, successorId))
          {
            return true;
          }
        }
      }
    }
    return false; // we did not find a (successor of a) a successor that leaves the intersection
  }
  else
  {
    // laneId is outside of the intersection, thus the route left the intersection
    mOutgoingLanesOnRoute.insert(laneId);
    return true;
  }
  return false;
}

lane::LaneIdSet
Intersection::successorsOnRouteLeavingIntersection(route::FullRoute const &route,
                                                   route::RoadSegmentList::const_iterator const &roadSegmentIt,
                                                   route::LaneSegment const &laneSegment)
{
  lane::LaneIdSet result;
  for (auto successorId : laneSegment.successors)
  {
    if (segmentLeavesIntersectionOnRoute(route, roadSegmentIt + 1, successorId))
    {
      result.insert(successorId);
    }
  }
  return result;
}

lane::LaneIdSet Intersection::successorsOnRouteLeavingIntersection(route::LaneSegment const &laneSegment)
{
  lane::LaneIdSet result;
  for (auto successorId : laneSegment.successors)
  {
    auto allReachableInternalAndOutgoingLanes
      = getAllReachableInternalAndOutgoingLanes(successorId, SuccessorMode::AnyIntersection);
    mLanesOnRoute.insert(allReachableInternalAndOutgoingLanes.first.begin(),
                         allReachableInternalAndOutgoingLanes.first.end());
    mOutgoingLanesOnRoute.insert(allReachableInternalAndOutgoingLanes.second.begin(),
                                 allReachableInternalAndOutgoingLanes.second.end());
    if (!allReachableInternalAndOutgoingLanes.second.empty())
    {
      result.insert(successorId);
    }
  }
  return result;
}

void Intersection::extractRightOfWayAndCollectTrafficLights(route::LaneInterval const &laneInterval,
                                                            lane::LaneIdSet const &successors,
                                                            lane::LaneId &toLaneId)
{
  auto beforeLaneId = laneInterval.laneId;
  bool const useSuccessor = (laneInterval.start < laneInterval.end);
  for (auto succId : successors)
  {
    auto intersectionType = getRightOfWayForTransition(beforeLaneId, succId, useSuccessor);
    if ((mIntersectionType != IntersectionType::Unknown) && (mIntersectionType != intersectionType))
    {
      access::getLogger()->warn("Different types of intersection detected! From {} To {}", beforeLaneId, succId);
    }
    toLaneId = succId;
    mIntersectionType = intersectionType;
    collectTrafficLights(beforeLaneId, toLaneId, useSuccessor);
  }
}

Intersection::Intersection(route::FullRoute const &route,
                           route::RoadSegmentList::const_iterator const &lastSegmentBeforeIntersection,
                           route::RoadSegmentList::const_iterator const &firstSegmentWithinIntersection)
  : mRoutePlanningCounter(route.routePlanningCounter)
  , mSegmentCountFromDestination(firstSegmentWithinIntersection->segmentCountFromDestination)
  , mSpeedLimit(std::numeric_limits<physics::Speed>::max())
{
  // ensure all intersection arms are initialized, to be safe when later accessing with std::map::at()
  mIntersectionArms[TurnDirection::Unknown] = lane::LaneIdSet();
  mIntersectionArms[TurnDirection::Right] = lane::LaneIdSet();
  mIntersectionArms[TurnDirection::Straight] = lane::LaneIdSet();
  mIntersectionArms[TurnDirection::Left] = lane::LaneIdSet();
  mIntersectionArms[TurnDirection::UTurn] = lane::LaneIdSet();

  // we don't need to check if the iterators are valid because this is called
  // from within getIntersectionForRoadSegment() only if the checks succeed
  lane::LaneId toLaneId{0};
  for (auto const &segment : lastSegmentBeforeIntersection->drivableLaneSegments)
  {
    mIncomingLanesOnRoute.insert(segment.laneInterval.laneId);
    auto successors = successorsOnRouteLeavingIntersection(route, lastSegmentBeforeIntersection, segment);
    extractRightOfWayAndCollectTrafficLights(segment.laneInterval, successors, toLaneId);
  }
  if (toLaneId == lane::LaneId(0))
  {
    // seems as if the route doesn't lead out of the intersection
    // so try to collect all possible ways out
    for (auto const &segment : lastSegmentBeforeIntersection->drivableLaneSegments)
    {
      auto successors = successorsOnRouteLeavingIntersection(segment);
      extractRightOfWayAndCollectTrafficLights(segment.laneInterval, successors, toLaneId);
    }
  }

  extractLanesOfIntersection(toLaneId);
  if (!laneIsPartOfIntersection(toLaneId))
  {
    throw std::runtime_error("Lane is not part of intersection!");
  }
  extractCrossingLanes();
  calculateParaPoints();
  orderIntersectionArmsAndExtractTurnDirection();
  extractLanesWithHigherPriority();
  extractLanesWithLowerPriority();
  extractLanesFromSameIntersectionArm();
  calculateEnteringProrityParaPoints();
  calculateSpeedLimit();
}

IntersectionType Intersection::intersectionType() const
{
  return mIntersectionType;
}

TurnDirection Intersection::turnDirection() const
{
  return mTurnDirection;
}

landmark::LandmarkIdSet const &Intersection::applicableTrafficLights() const
{
  return mTrafficLightIds;
}

lane::LaneIdSet const &Intersection::lanesOnRoute() const
{
  return mLanesOnRoute;
}

point::ParaPointList const &Intersection::paraPointsOnRoute() const
{
  return mParaPointsOnRoute;
}

lane::LaneIdSet const &Intersection::incomingLanesOnRoute() const
{
  return mIncomingLanesOnRoute;
}

lane::LaneIdSet const &Intersection::outgoingLanesOnRoute() const
{
  return mOutgoingLanesOnRoute;
}

point::ParaPointList const &Intersection::incomingParaPointsOnRoute() const
{
  return mIncomingParaPointsOnRoute;
}

point::ParaPointList const &Intersection::outgoingParaPointsOnRoute() const
{
  return mOutgoingParaPointsOnRoute;
}

point::ParaPointList const &Intersection::outgoingParaPoints() const
{
  return mOutgoingParaPoints;
}

lane::LaneIdSet const &Intersection::internalLanes() const
{
  return mInternalLanes;
}

lane::LaneIdSet const &Intersection::internalLanesWithHigherPriority() const
{
  return mInternalLanesWithHigherPriority;
}

lane::LaneIdSet const &Intersection::internalLanesWithLowerPriority() const
{
  return mInternalLanesWithLowerPriority;
}

lane::LaneIdSet const &Intersection::incomingLanesWithLowerPriority() const
{
  return mIncomingLanesWithLowerPriority;
}

lane::LaneIdSet const &Intersection::incomingLanesWithHigherPriority() const
{
  return mIncomingLanesWithHigherPriority;
}

lane::LaneIdSet const &Intersection::incomingLanes() const
{
  return mIncomingLanes;
}

lane::LaneIdSet const &Intersection::outgoingLanes() const
{
  return mOutgoingLanes;
}

lane::LaneIdSet const &Intersection::entryLanes() const
{
  return mEntryLanes;
}

point::ParaPointList const &Intersection::incomingParaPoints() const
{
  return mIncomingParaPoints;
}

point::ParaPointList const &Intersection::entryParaPoints() const
{
  return mEntryParaPoints;
}

point::ParaPointList const &Intersection::incomingParaPointsWithHigherPriority() const
{
  return mIncomingParaPointsWithHigherPriority;
}

point::ParaPointList const &Intersection::incomingParaPointsWithLowerPriority() const
{
  return mIncomingParaPointsWithLowerPriority;
}

lane::LaneIdSet const &Intersection::crossingLanes() const
{
  return mCrossingLanes;
}

route::RoutePlanningCounter Intersection::getRoutePlanningCounter() const
{
  return mRoutePlanningCounter;
}

route::SegmentCounter Intersection::getRouteSegmentCountFromDestination() const
{
  return mSegmentCountFromDestination;
}

void Intersection::updateRouteCounters(route::RoutePlanningCounter newRoutePlanningCounter,
                                       route::SegmentCounter newRouteSegmentCounter)
{
  mRoutePlanningCounter = newRoutePlanningCounter;
  mSegmentCountFromDestination = newRouteSegmentCounter;
}

route::RouteParaPoint Intersection::getIntersectionStartOnRoute() const
{
  route::RouteParaPoint result;
  result.routePlanningCounter = getRoutePlanningCounter();
  result.segmentCountFromDestination = getRouteSegmentCountFromDestination();
  result.parametricOffset = physics::ParametricValue(0.);
  return result;
}

bool Intersection::laneIsPartOfIntersection(lane::LaneId const laneId) const
{
  return (mInternalLanes.count(laneId) > 0);
}

void Intersection::extractLanesOfIntersection(lane::LaneId const laneId)
{
  if (laneIsPartOfIntersection(laneId))
  {
    return;
  }
  auto lane = lane::getLane(laneId);
  if (!lane::isLanePartOfAnIntersection(lane))
  {
    // restrict ourselves to lanes inside the intersection
    return;
  }
  mInternalLanes.insert(laneId);
  for (auto const &contact : lane.contactLanes)
  {
    processContactsForLane(lane, contact);
    extractLanesOfIntersection(contact.toLane);
  }
}

void Intersection::processContactsForLane(lane::Lane const &lane, lane::ContactLane const &contact)
{
  auto const &toId = contact.toLane;
  switch (contact.location)
  {
    case lane::ContactLocation::OVERLAP:
      mOverlapping[lane.id].insert(toId);
      break;
    case lane::ContactLocation::SUCCESSOR:
      if (isLaneDirectionPositive(lane))
      {
        mSuccessor[lane.id].insert(toId);
        insertOutgoingLane(toId);
      }
      else
      {
        mPredecessor[lane.id].insert(toId);
        insertIncomingLane(toId);
      }
      break;
    case lane::ContactLocation::PREDECESSOR:
      if (isLaneDirectionNegative(lane))
      {
        mSuccessor[lane.id].insert(toId);
        insertOutgoingLane(toId);
      }
      else
      {
        mPredecessor[lane.id].insert(toId);
        insertIncomingLane(toId);
      }
      break;
    default:
      break; // ignored for now
  }
}

void Intersection::insertIncomingLane(lane::LaneId const laneId)
{
  if (!isLanePartOfAnIntersection(laneId))
  {
    mEntryLanes.insert(laneId);
    if (mIncomingLanesOnRoute.count(laneId) == 0)
    {
      mIncomingLanes.insert(laneId);
    }
  }
}

void Intersection::extractCrossingLanes()
{
  // first, collect all lanes that overlap with any lane of the route
  for (auto insideId : mLanesOnRoute)
  {
    mCrossingLanes.insert(mOverlapping[insideId].begin(), mOverlapping[insideId].end());
  }
  // remove all lanes that are a successor of one of the incoming lanes on the route
  // otherwise vehicles driving in the same or parallel direction would also be considered...
  lane::LaneIdSet internalSuccessorsOfRouteEntry;
  for (auto laneId : mIncomingLanesOnRoute)
  {
    auto succ = getAllSuccessorsInLaneDirectionWithinIntersection(laneId, SuccessorMode::OwnIntersection);
    internalSuccessorsOfRouteEntry.insert(succ.begin(), succ.end());
  }
  for (auto laneId : internalSuccessorsOfRouteEntry)
  {
    mCrossingLanes.erase(laneId);
  }
}

void Intersection::insertOutgoingLane(lane::LaneId const laneId)
{
  if (!isLanePartOfAnIntersection(laneId) && (mIncomingLanes.count(laneId) == 0))
  {
    if (mOutgoingLanes.insert(laneId).second)
    {
      point::ParaPoint p;
      p.laneId = laneId;
      p.parametricOffset
        = (isLaneDirectionNegative(laneId) ? physics::ParametricValue(0.) : physics::ParametricValue(1.));
      mOutgoingParaPoints.push_back(p);
    }
  }
}

// this function return the directional angle of laneId
point::ENUHeading getLaneDirectionalAngle(lane::LaneId laneId)
{
  auto lane = lane::getLane(laneId);

  // Collect the parapoint of the lane
  physics::ParametricValue laneParapoint
    = (isLaneDirectionNegative(lane) ? physics::ParametricValue(0.) : physics::ParametricValue(1.));

  // get the starting point and end point of the lane
  point::ECEFPoint laneStartPoint = getParametricPoint(lane, laneParapoint, physics::ParametricValue(0.5));
  point::ECEFPoint laneEndPoint
    = getParametricPoint(lane, physics::ParametricValue(1.) - laneParapoint, physics::ParametricValue(0.5));

  // get the directional vector
  point::ECEFHeading directionalVector = point::createECEFHeading(laneStartPoint, laneEndPoint);
  point::ENUHeading headingENU = point::createENUHeading(directionalVector, laneStartPoint);

  return headingENU;
}

static void directionOfLanes(point::ParaPointList const &lanes,
                             point::ENUHeading referenceAngle,
                             std::vector<std::pair<point::ENUHeading, point::ParaPoint>> &intersectionLanesDirection)

{
  for (auto const &lane : lanes)
  {
    auto angle = getLaneDirectionalAngle(lane.laneId);

    // all angles should be relative to the angle of the arm where the route enters the intersection.
    angle = angle - referenceAngle;
    if (angle < point::ENUHeading(0))
    {
      angle += point::ENUHeading(2 * M_PI);
    }
    intersectionLanesDirection.push_back({angle, lane});
  }
}

point::ENUHeading degToENUHeading(double angle)
{
  return point::ENUHeading(angle * M_PI / 180.);
}

TurnDirection turnDirectionForAngle(point::ENUHeading const angle)
{
  // we assume that the angle is in range [0 2 PI] where 0 is the direction of the entering lane
  if ((angle > degToENUHeading(315.)) || (angle < degToENUHeading(45.)))
  {
    return TurnDirection::UTurn;
  }
  else if (angle > degToENUHeading(225.))
  {
    return TurnDirection::Left;
  }
  else if (angle > degToENUHeading(135.))
  {
    return TurnDirection::Straight;
  }
  else if (angle > degToENUHeading(45.))
  {
    return TurnDirection::Right;
  }
  else
  {
    // we should never come here
    return TurnDirection::Unknown;
  }
}

void Intersection::orderIntersectionArmsAndExtractTurnDirection()
{
  std::vector<std::pair<point::ENUHeading, point::ParaPoint>> intersectionLanesDirection;

  auto const egoIntersectionEntryLaneId = *mIncomingLanesOnRoute.begin();

  // angle (around up-axis) of the incoming lane, used as reference angle
  auto const referenceAngle = getLaneDirectionalAngle(egoIntersectionEntryLaneId);

  // calculate the angle for all incoming lanes of the intersection
  directionOfLanes(mEntryParaPoints, referenceAngle, intersectionLanesDirection);

  // calculate the angle for all outgoing lanes of the intersection
  directionOfLanes(mOutgoingParaPoints, (referenceAngle + point::ENUHeading(M_PI)), intersectionLanesDirection);

  // Group all incoming/outgoing lanes into arms (ordered counterclockwise).

  for (auto laneDirection : intersectionLanesDirection)
  {
    auto turnDirection = turnDirectionForAngle(laneDirection.first);
    if ((mTurnDirection == TurnDirection::Unknown) && (mOutgoingLanesOnRoute.count(laneDirection.second.laneId) > 0))
    {
      mTurnDirection = turnDirection;
    }
    mIntersectionArms[turnDirection].insert(laneDirection.second.laneId);
  }
}

void Intersection::extractLanesWithHigherPriority()
{
  switch (mIntersectionType)
  {
    case IntersectionType::Yield: // fallthrough intended here
    case IntersectionType::Stop:
      adjustLanesForYield();
      break;
    case IntersectionType::Unknown: // fallthrough intended here, unknown means be careful with crossing lanes
    case IntersectionType::AllWayStop:
      // nothing to be done, simply use the crossing lanes
      mInternalLanesWithHigherPriority = mCrossingLanes;
      break;
    case IntersectionType::HasWay:
      adjustLanesForHasWay();
      break;
    case IntersectionType::Crosswalk: // no need for adaptation, there should not be any crossing lanes
      break;
    case IntersectionType::PriorityToRight:
    case IntersectionType::PriorityToRightAndStraight:
      adjustPriorityToRight();
      break;
    case IntersectionType::TrafficLight:
      adjustLanesForTrafficLight();
      break;
    default:
      // throw exception?
      break;
  }
}

void Intersection::extractLanesWithLowerPriority()
{
  for (auto laneId : mInternalLanes)
  {
    if ((mLanesOnRoute.find(laneId) == mLanesOnRoute.end())
        && (mInternalLanesWithHigherPriority.find(laneId) == mInternalLanesWithHigherPriority.end()))
    {
      mInternalLanesWithLowerPriority.insert(laneId);
    }
  }
}

void Intersection::extractLanesFromSameIntersectionArm()
{
  for (auto laneId : mIncomingLanesOnRoute)
  {
    auto relevantLanes = getAllSuccessorsInLaneDirectionWithinIntersection(laneId, SuccessorMode::OwnIntersection);
    mInternalLanesFromSameIntersectionArm.insert(relevantLanes.begin(), relevantLanes.end());
  }
}

void Intersection::calculateParaPoints()
{
  for (auto laneId : mIncomingLanesOnRoute)
  {
    mIncomingParaPointsOnRoute.push_back(getIncomingParaPoint(laneId));
  }

  for (auto outGoingLaneId : mOutgoingLanesOnRoute)
  {
    mOutgoingParaPointsOnRoute.push_back(getOutgoingParaPoint(outGoingLaneId));
  }

  for (auto laneId : mIncomingLanes)
  {
    auto paraPoint = getIncomingParaPoint(laneId);
    mIncomingParaPoints.push_back(paraPoint);
  }

  for (auto laneId : mEntryLanes)
  {
    auto paraPoint = getIncomingParaPoint(laneId);
    mEntryParaPoints.push_back(paraPoint);
  }

  for (auto laneId : mLanesOnRoute)
  {
    auto paraPoint = getIncomingParaPoint(laneId);
    mParaPointsOnRoute.push_back(paraPoint);
  }
}

void Intersection::calculateEnteringProrityParaPoints()
{
  for (auto laneId : mIncomingLanes)
  {
    auto successorLanes = getAllSuccessorsInLaneDirectionWithinIntersection(laneId, SuccessorMode::OwnIntersection);
    bool internalPriorityFound = false;

    for (auto successorLaneIter = successorLanes.begin();
         successorLaneIter != successorLanes.end() && !internalPriorityFound;
         successorLaneIter++)
    {
      if (mInternalLanesWithHigherPriority.find(*successorLaneIter) != mInternalLanesWithHigherPriority.end())
      {
        internalPriorityFound = true;
      }
    }
    if (internalPriorityFound)
    {
      auto paraPoint = getIncomingParaPoint(laneId);
      mIncomingParaPointsWithHigherPriority.push_back(paraPoint);
      mIncomingLanesWithHigherPriority.insert(laneId);
    }
    else
    {
      auto paraPoint = getIncomingParaPoint(laneId);
      mIncomingParaPointsWithLowerPriority.push_back(paraPoint);
      mIncomingLanesWithLowerPriority.insert(laneId);
    }
  }
}

bool Intersection::turnDirectionCrossesStraightTraffic() const
{
  // right handed traffic
  if (access::isRightHandedTraffic())
  {
    return (mTurnDirection == TurnDirection::Left) || (mTurnDirection == TurnDirection::UTurn);
  }
  // left handed traffic
  else
  {
    return (mTurnDirection == TurnDirection::Right) || (mTurnDirection == TurnDirection::UTurn);
  }
}

bool Intersection::outgoingIntersectionArmCanBeReached(lane::LaneId const laneId,
                                                       TurnDirection const outgoingIntersectionArm)
{
  for (auto reachableOutgoingLaneId : getAllReachableOutgoingLanes(laneId, SuccessorMode::OwnIntersection))
  {
    if (mIntersectionArms[outgoingIntersectionArm].count(reachableOutgoingLaneId) > 0)
    {
      return true;
    }
  }
  return false;
}

void Intersection::addLaneAndSuccessorsToInternalLanesWithHigherPriorityIfCrossing(
  lane::LaneId const &lane, TurnDirection const restrictToOutgoingIntersectionArm)
{
  auto lanesToCheck = getLaneAndAllSuccessorsInLaneDirectionWithinIntersection(lane, SuccessorMode::OwnIntersection);
  for (auto laneToCheckId : lanesToCheck)
  {
    if (mCrossingLanes.count(laneToCheckId) > 0)
    {
      if ((restrictToOutgoingIntersectionArm == TurnDirection::Unknown)
          || outgoingIntersectionArmCanBeReached(laneToCheckId, restrictToOutgoingIntersectionArm))
      {
        mInternalLanesWithHigherPriority.insert(laneToCheckId);
      }
    }
  }
}

void Intersection::addLaneAndSuccessorsToInternalLanesWithHigherPriorityIfCrossing(
  lane::LaneIdSet const &lanes, TurnDirection const restrictToOutgoingIntersectionArm)
{
  for (auto laneId : lanes)
  {
    addLaneAndSuccessorsToInternalLanesWithHigherPriorityIfCrossing(laneId, restrictToOutgoingIntersectionArm);
  }
}

void Intersection::addLaneAndSuccessorsToInternalLanesWithHigherPriority(
  lane::LaneId const &lane, TurnDirection const restrictToOutgoingIntersectionArm)
{
  auto lanesToCheck = getLaneAndAllSuccessorsInLaneDirectionWithinIntersection(lane, SuccessorMode::OwnIntersection);
  for (auto laneToCheckId : lanesToCheck)
  {
    if ((restrictToOutgoingIntersectionArm == TurnDirection::Unknown)
        || outgoingIntersectionArmCanBeReached(laneToCheckId, restrictToOutgoingIntersectionArm))
    {
      mInternalLanesWithHigherPriority.insert(laneToCheckId);
    }
  }
}

void Intersection::addLaneAndSuccessorsToInternalLanesWithHigherPriority(
  lane::LaneIdSet const &lanes, TurnDirection const restrictToOutgoingIntersectionArm)
{
  for (auto laneId : lanes)
  {
    addLaneAndSuccessorsToInternalLanesWithHigherPriority(laneId, restrictToOutgoingIntersectionArm);
  }
}

void Intersection::adjustLanesForHasWay()
{
  for (auto const &intersectionArm : mIntersectionArms)
  {
    for (auto laneId : intersectionArm.second)
    {
      auto sucessorLanes = getDirectSuccessorsInLaneDirectionWithinIntersection(laneId, SuccessorMode::OwnIntersection);
      for (auto sucessorLaneId : sucessorLanes)
      {
        auto trafficRuleAtOtherLane
          = getRightOfWayForTransition(laneId, sucessorLaneId, !isLaneDirectionNegative(laneId));

        // has way lanes have to be respected if we turn
        if ((trafficRuleAtOtherLane == IntersectionType::HasWay) && turnDirectionCrossesStraightTraffic())
        {
          addLaneAndSuccessorsToInternalLanesWithHigherPriorityIfCrossing(sucessorLaneId);
        }
      }
    }
  }
}

void Intersection::adjustLanesForYield()
{
  for (auto const &intersectionArm : mIntersectionArms)
  {
    for (auto laneId : intersectionArm.second)
    {
      auto sucessorLanes = getDirectSuccessorsInLaneDirectionWithinIntersection(laneId, SuccessorMode::OwnIntersection);
      for (auto sucessorLaneId : sucessorLanes)
      {
        auto trafficRuleAtOtherLane
          = getRightOfWayForTransition(laneId, sucessorLaneId, !isLaneDirectionNegative(laneId));
        // need to give way to all of lanes that have way
        if (trafficRuleAtOtherLane == IntersectionType::HasWay)
        {
          addLaneAndSuccessorsToInternalLanesWithHigherPriority(sucessorLaneId);
        }
        // yield lanes only have to be respected if we turn
        else if ((trafficRuleAtOtherLane == IntersectionType::Yield) && turnDirectionCrossesStraightTraffic())
        {
          addLaneAndSuccessorsToInternalLanesWithHigherPriorityIfCrossing(sucessorLaneId);
        }
      }
    }
  }
}

void Intersection::adjustPriorityToRight()
{
  if ((IntersectionType::PriorityToRightAndStraight == mIntersectionType)
      && (mTurnDirection == TurnDirection::Straight))
  {
    // if we drive straight, only the straight and left turning has prio
    addLaneAndSuccessorsToInternalLanesWithHigherPriority(mIntersectionArms[TurnDirection::Right], TurnDirection::Left);
    addLaneAndSuccessorsToInternalLanesWithHigherPriority(mIntersectionArms[TurnDirection::Right],
                                                          TurnDirection::UTurn);
  }
  else
  {
    // all traffic from right
    addLaneAndSuccessorsToInternalLanesWithHigherPriority(mIntersectionArms[TurnDirection::Right]);
  }

  // plus take care on the straight traffic when turning
  if (turnDirectionCrossesStraightTraffic())
  {
    addLaneAndSuccessorsToInternalLanesWithHigherPriorityIfCrossing(mIntersectionArms[TurnDirection::Straight]);
    if (IntersectionType::PriorityToRightAndStraight == mIntersectionType)
    {
      // the other straight driving lanes have priority, too
      addLaneAndSuccessorsToInternalLanesWithHigherPriorityIfCrossing(mIntersectionArms[TurnDirection::Right],
                                                                      TurnDirection::Left);
      addLaneAndSuccessorsToInternalLanesWithHigherPriorityIfCrossing(mIntersectionArms[TurnDirection::Left],
                                                                      TurnDirection::Right);
    }
  }
}

void Intersection::adjustLanesForTrafficLight()
{
  if (turnDirectionCrossesStraightTraffic() && onlySolidTrafficLightsOnRoute())
  {
    addLaneAndSuccessorsToInternalLanesWithHigherPriorityIfCrossing(mIntersectionArms[TurnDirection::Straight]);
  }
}

bool Intersection::objectWithinIntersection(match::MapMatchedObjectBoundingBox const &object) const
{
  for (auto const &occupiedLane : object.laneOccupiedRegions)
  {
    if (mInternalLanes.count(occupiedLane.laneId) > 0)
    {
      return true;
    }
  }
  return false;
}

bool Intersection::objectOnIncomingLane(match::MapMatchedObjectBoundingBox const &object) const
{
  for (auto const &occupiedLane : object.laneOccupiedRegions)
  {
    if (mIncomingLanes.count(occupiedLane.laneId) > 0)
    {
      return true;
    }
  }
  return false;
}

bool Intersection::objectOnInternalLaneWithLowerPriority(match::MapMatchedObjectBoundingBox const &object) const
{
  for (auto const &occupiedLane : object.laneOccupiedRegions)
  {
    if (mInternalLanesWithLowerPriority.count(occupiedLane.laneId) > 0)
    {
      return true;
    }
  }
  return false;
}

bool Intersection::objectOnIncomingLaneWithLowerPriority(match::MapMatchedObjectBoundingBox const &object) const
{
  for (auto const &occupiedLane : object.laneOccupiedRegions)
  {
    if (mIncomingLanesWithLowerPriority.count(occupiedLane.laneId) > 0)
    {
      return true;
    }
  }
  return false;
}

bool Intersection::objectOnLaneWithLowerPriority(match::MapMatchedObjectBoundingBox const &object) const
{
  return objectOnInternalLaneWithLowerPriority(object) || objectOnIncomingLaneWithLowerPriority(object);
}

bool Intersection::objectOnInternalLaneWithHigherPriority(match::MapMatchedObjectBoundingBox const &object) const
{
  for (auto const &occupiedLane : object.laneOccupiedRegions)
  {
    if (mInternalLanesWithHigherPriority.count(occupiedLane.laneId) > 0)
    {
      return true;
    }
  }
  return false;
}

bool Intersection::objectOnIncomingLaneWithHigherPriority(match::MapMatchedObjectBoundingBox const &object) const
{
  for (auto const &occupiedLane : object.laneOccupiedRegions)
  {
    if (mIncomingLanesWithHigherPriority.count(occupiedLane.laneId) > 0)
    {
      return true;
    }
  }
  return false;
}

bool Intersection::objectOnLaneWithHigherPriority(match::MapMatchedObjectBoundingBox const &object) const
{
  return objectOnInternalLaneWithHigherPriority(object) || objectOnIncomingLaneWithHigherPriority(object);
}

bool Intersection::objectOnCrossingLane(match::MapMatchedObjectBoundingBox const &object) const
{
  for (auto const &occupiedLane : object.laneOccupiedRegions)
  {
    if (mCrossingLanes.count(occupiedLane.laneId) > 0)
    {
      return true;
    }
  }
  return false;
}

bool Intersection::objectOnIntersectionRoute(match::MapMatchedObjectBoundingBox const &object) const
{
  for (auto const &occupiedLane : object.laneOccupiedRegions)
  {
    if (mLanesOnRoute.count(occupiedLane.laneId) > 0)
    {
      return true;
    }
  }
  return false;
}

bool Intersection::objectRouteCrossesIntersectionRoute(route::FullRoute const &objectRoute) const
{
  for (auto crossingLaneId : mCrossingLanes)
  {
    auto findResult = route::findWaypoint(crossingLaneId, objectRoute);
    if (findResult.isValid())
    {
      return true;
    }
  }
  return false;
}

bool Intersection::objectRouteCrossesIntersection(route::FullRoute const &objectRoute) const
{
  for (auto internalLaneId : mInternalLanes)
  {
    auto findResult = route::findWaypoint(internalLaneId, objectRoute);
    if (findResult.isValid())
    {
      return true;
    }
  }
  return false;
}

bool Intersection::objectRouteFromSameArmAsIntersectionRoute(route::FullRoute const &objectRoute) const
{
  bool routeLaneFound = false;
  for (auto const &roadSegment : objectRoute.roadSegments)
  {
    for (auto const &laneSegment : roadSegment.drivableLaneSegments)
    {
      if ((mInternalLanesFromSameIntersectionArm.count(laneSegment.laneInterval.laneId) > 0)
          || (mIncomingLanesOnRoute.count(laneSegment.laneInterval.laneId) > 0))
      {
        routeLaneFound = true;
      }
      else if (mInternalLanes.count(laneSegment.laneInterval.laneId) > 0)
      {
        // another internal lane is found in the route, so route differs
        return false;
      }
      else if (routeLaneFound)
      {
        // if we previously have found a route lane and are now again outside the intersection: route is identical
        return routeLaneFound;
      }
      else
      {
        // nothing decided yet since neither a lane on intersection route found nor any other internal lane touched
      }
    }
  }
  return routeLaneFound;
}

bool Intersection::objectRouteOppositeToIntersectionRoute(route::FullRoute const &objectRoute) const
{
  route::FindWaypointResult findResult(objectRoute);
  for (auto const &laneOnOutgoingIntersectionArm : mIntersectionArms.at(mTurnDirection))
  {
    findResult = route::findWaypoint(laneOnOutgoingIntersectionArm, objectRoute);
    if (findResult.isValid())
    {
      break;
    }
  }
  if (!findResult.isValid())
  {
    // check if the object route starts already within the intersection
    if (!objectRoute.roadSegments.empty() && !objectRoute.roadSegments.begin()->drivableLaneSegments.empty()
        && mInternalLanes.count(objectRoute.roadSegments.begin()->drivableLaneSegments.begin()->laneInterval.laneId)
          > 0)
    {
      // object is already within intersection
      for (auto const &laneOnOutgoingIntersectionArm : mIntersectionArms.at(mTurnDirection))
      {
        auto internalLanesFromOutgoingIntersectionArm = getAllSuccessorsInLaneDirectionWithinIntersection(
          laneOnOutgoingIntersectionArm, SuccessorMode::OwnIntersection);
        if (internalLanesFromOutgoingIntersectionArm.count(
              objectRoute.roadSegments.begin()->drivableLaneSegments.begin()->laneInterval.laneId)
            > 0)
        {
          // and object came from outgoing intersection arm
          findResult = route::findWaypoint(
            objectRoute.roadSegments.begin()->drivableLaneSegments.begin()->laneInterval.laneId, objectRoute);
        }
      }
    }
  }

  if (findResult.isValid())
  {
    // shorten object route
    route::FullRoute remainingObjectRoute(objectRoute);
    route::shortenRoute(findResult.queryPosition, remainingObjectRoute);

    // now search for leaving the intersection at the route incoming arm
    for (auto const &laneOnIncomingIntersectionArm : mIntersectionArms.at(TurnDirection::UTurn))
    {
      auto result = route::findWaypoint(laneOnIncomingIntersectionArm, remainingObjectRoute);
      if (result.isValid())
      {
        return true;
      }
    }
  }
  return false;
}

physics::Distance Intersection::objectDistanceToIntersection(match::Object const &object) const
{
  physics::Distance minDistance = std::numeric_limits<physics::Distance>::max();
  if (objectWithinIntersection(object.mapMatchedBoundingBox))
  {
    minDistance = physics::Distance(0.);
  }
  else
  {
    for (auto internalLaneId : mInternalLanes)
    {
      minDistance = std::min(minDistance, lane::getDistanceToLane(internalLaneId, object));
    }
  }
  return minDistance;
}

physics::Distance Intersection::objectInterpenetrationDistanceWithIntersection(match::Object const &object) const
{
  physics::Distance maxDistance(0.);
  TurnDirection coveredIntersectionArms = TurnDirection::Unknown;
  for (auto const &occupiedLane : object.mapMatchedBoundingBox.laneOccupiedRegions)
  {
    if (mInternalLanes.count(occupiedLane.laneId) > 0u)
    {
      // internal lane
      // if this lane is at the intersection border and the object is still touching that border
      // then the range covered within this lane has to be considered for the inter penetration distances
      bool isRelevantBorderLane = false;
      if ((occupiedLane.longitudinalRange.maximum == physics::ParametricValue(1.))
          || (occupiedLane.longitudinalRange.minimum == physics::ParametricValue(0.)))
      {
        // check predecessors
        auto findPredecessors = mPredecessor.find(occupiedLane.laneId);
        if (findPredecessors != mPredecessor.end())
        {
          for (auto predecessorId : findPredecessors->second)
          {
            if (!isLanePartOfAnIntersection(predecessorId))
            {
              // one of the predecessors is outside of the intersection (outgoingParaPonts logic)
              auto const entryBorderParaPoint = getOutgoingParaPoint(occupiedLane.laneId);
              if ((entryBorderParaPoint.parametricOffset == occupiedLane.longitudinalRange.maximum)
                  || (entryBorderParaPoint.parametricOffset == occupiedLane.longitudinalRange.minimum))
              {
                isRelevantBorderLane = true;
                break;
              }
            }
          }
        }
        if (!isRelevantBorderLane)
        {
          // check successors
          auto findSuccessors = mSuccessor.find(occupiedLane.laneId);
          if (findSuccessors != mSuccessor.end())
          {
            for (auto successorId : findSuccessors->second)
            {
              if (!isLanePartOfAnIntersection(successorId))
              {
                // one of the successors is outside of the intersection (incomingParaPonts logic)
                auto const entryBorderParaPoint = getIncomingParaPoint(occupiedLane.laneId);
                if ((entryBorderParaPoint.parametricOffset == occupiedLane.longitudinalRange.maximum)
                    || (entryBorderParaPoint.parametricOffset == occupiedLane.longitudinalRange.minimum))
                {
                  isRelevantBorderLane = true;
                  break;
                }
              }
            }
          }
        }
      }

      if (isRelevantBorderLane)
      {
        auto const laneLength = lane::getLane(occupiedLane.laneId).length;
        physics::Distance const penetrationDistance
          = (occupiedLane.longitudinalRange.maximum - occupiedLane.longitudinalRange.minimum) * laneLength;
        maxDistance = std::max(maxDistance, penetrationDistance);
      }
      else
      {
        // otherwise this lane is touched as part of an actual internal intersection part and
        // we only have to ensure that max-distance becomes > 0.
        // and so, if no outgoing lane is touched (i.e. no intersection arm),
        // the case fully within intersection is applied
        maxDistance = std::max(maxDistance, physics::Distance(0.1));
      }
    }
    else
    {
      // not an internal lane, check if different intersection arms covered by the vehicle
      for (auto const &intersectionArm : mIntersectionArms)
      {
        if (intersectionArm.second.count(occupiedLane.laneId) > 0u)
        {
          if (coveredIntersectionArms == TurnDirection::Unknown)
          {
            coveredIntersectionArms = intersectionArm.first;
          }
          else if (coveredIntersectionArms != intersectionArm.first)
          {
            // covering two different intersection arms, actually crossing the intersection
            return object.enuPosition.dimension.length;
          }
        }
      }
    }
  }

  if ((maxDistance > physics::Distance(0.)) && (coveredIntersectionArms == TurnDirection::Unknown))
  {
    // not touching incoming/outgoing lane at all, but touching an internal lane
    // => fully within intersection
    maxDistance = object.enuPosition.dimension.length;
  }

  // result must not extend length of the object
  maxDistance = std::min(maxDistance, object.enuPosition.dimension.length);
  return maxDistance;
}

bool Intersection::objectRouteCrossesLanesWithHigherPriority(route::FullRoute const &objectRoute) const
{
  for (auto laneWithHigherPrioId : mInternalLanesWithHigherPriority)
  {
    auto findResult = route::findWaypoint(laneWithHigherPrioId, objectRoute);
    if (findResult.isValid())
    {
      return true;
    }
  }

  return false;
}

void Intersection::collectTrafficLights(lane::LaneId fromLaneId, lane::LaneId toLaneId, bool useSuccessor)
{
  // keep the first match, we just need any match
  auto fromLane = lane::getLane(fromLaneId);
  auto location = lane::ContactLocation::PREDECESSOR;
  if (useSuccessor)
  {
    location = lane::ContactLocation::SUCCESSOR;
  }
  auto contacts = getContactLanes(fromLane, location);
  for (auto const &contact : contacts)
  {
    if (contact.toLane == toLaneId)
    {
      for (auto contactType : contact.types)
      {
        if (contactType == lane::ContactType::TRAFFIC_LIGHT)
        {
          mTrafficLightIds.insert(contact.trafficLightId);
        }
      }
    }
  }
}

physics::Speed Intersection::getSpeedLimit() const
{
  return mSpeedLimit;
}

void Intersection::calculateSpeedLimit()
{
  for (auto const &internalLaneId : mInternalLanes)
  {
    route::LaneInterval laneInterval;
    laneInterval.laneId = internalLaneId;
    laneInterval.start = physics::ParametricValue(0.);
    laneInterval.end = physics::ParametricValue(1.);
    auto speedLimits = route::getSpeedLimits(laneInterval);
    for (auto const &speedLimit : speedLimits)
    {
      mSpeedLimit = std::min(mSpeedLimit, speedLimit.speedLimit);
    }
  }
}

landmark::TrafficLightType Intersection::extractTrafficLightType(landmark::LandmarkId trafficLightId)
{
  auto trafficLight = landmark::getLandmark(trafficLightId);
  return trafficLight.trafficLightType;
}

bool Intersection::isSolidTrafficLight(landmark::LandmarkId trafficLightId)
{
  switch (extractTrafficLightType(trafficLightId))
  {
    case landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_GREEN:
    case landmark::TrafficLightType::BIKE_PEDESTRIAN_RED_YELLOW_GREEN:
    case landmark::TrafficLightType::BIKE_RED_GREEN:
    case landmark::TrafficLightType::BIKE_RED_YELLOW_GREEN:
    case landmark::TrafficLightType::PEDESTRIAN_RED_GREEN:
    case landmark::TrafficLightType::PEDESTRIAN_RED_YELLOW_GREEN:
    case landmark::TrafficLightType::RIGHT_RED_YELLOW_GREEN:
    case landmark::TrafficLightType::RIGHT_STRAIGHT_RED_YELLOW_GREEN:
    case landmark::TrafficLightType::STRAIGHT_RED_YELLOW_GREEN:
    case landmark::TrafficLightType::LEFT_RED_YELLOW_GREEN:
    case landmark::TrafficLightType::LEFT_STRAIGHT_RED_YELLOW_GREEN:
      return false;
      break;
    case landmark::TrafficLightType::INVALID:
    case landmark::TrafficLightType::UNKNOWN:
    case landmark::TrafficLightType::SOLID_RED_YELLOW:
    case landmark::TrafficLightType::SOLID_RED_YELLOW_GREEN:
      return true;
      break;
    default:
      throw std::invalid_argument("Intersection isSolidTrafficLight: Invalid traffic light type given");
  }
}

bool Intersection::onlySolidTrafficLightsOnRoute()
{
  for (auto trafficLightId : mTrafficLightIds)
  {
    if (!isSolidTrafficLight(trafficLightId))
    {
      return false;
    }
  }
  return true;
}

} // namespace intersection
} // namespace map
} // namespace ad
