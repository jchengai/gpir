// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/route/RouteOperation.hpp"

#include <algorithm>
#include <map>
#include <set>

#include "RouteOperationPrivate.hpp"
#include "ad/map/access/Logging.hpp"
#include "ad/map/access/Operation.hpp"
#include "ad/map/intersection/Intersection.hpp"
#include "ad/map/lane/BorderOperation.hpp"
#include "ad/map/lane/LaneOperation.hpp"
#include "ad/map/match/MapMatchedOperation.hpp"
#include "ad/map/point/Operation.hpp"
#include "ad/map/route/LaneSegment.hpp"
#include "ad/map/route/Planning.hpp"

namespace ad {
namespace map {
namespace route {

RouteIterator getRouteIterator(route::RouteParaPoint const &routePosition, route::FullRoute const &route)
{
  RouteIterator result(route, route.roadSegments.end());
  if (routePosition.routePlanningCounter != route.routePlanningCounter)
  {
    // route position not matching the route
    return result;
  }

  if (route.roadSegments.empty()
      || (route.roadSegments.front().segmentCountFromDestination < routePosition.segmentCountFromDestination)
      || (route.roadSegments.back().segmentCountFromDestination > routePosition.segmentCountFromDestination))
  {
    // route position not found within route
    return result;
  }

  for (result.roadSegmentIterator = route.roadSegments.begin(); result.roadSegmentIterator != route.roadSegments.end();
       result.roadSegmentIterator++)
  {
    if (result.roadSegmentIterator->segmentCountFromDestination == routePosition.segmentCountFromDestination)
    {
      // found the segment
      break;
    }
  }
  return result;
}

point::ParaPoint getLaneParaPoint(physics::ParametricValue const &routeParametricOffset,
                                  route::LaneInterval const &laneInterval)
{
  point::ParaPoint lanePoint;
  if (isDegenerated(laneInterval))
  {
    lanePoint = getIntervalStart(laneInterval);
  }
  else
  {
    lanePoint.laneId = laneInterval.laneId;
    physics::ParametricValue const parametricLengthOffsetFromRouteStart
      = calcParametricLength(laneInterval) * routeParametricOffset;
    if (isRouteDirectionPositive(laneInterval))
    {
      lanePoint.parametricOffset = laneInterval.start + parametricLengthOffsetFromRouteStart;
    }
    else
    {
      lanePoint.parametricOffset = laneInterval.start - parametricLengthOffsetFromRouteStart;
    }
  }
  return lanePoint;
}

point::ParaPointList getLaneParaPoints(route::RouteParaPoint const &routePosition, route::FullRoute const &route)
{
  point::ParaPointList resultPoints;
  auto routeIter = getRouteIterator(routePosition, route);
  if (routeIter.isValid())
  {
    for (auto &laneSegment : routeIter.roadSegmentIterator->drivableLaneSegments)
    {
      resultPoints.push_back(getLaneParaPoint(routePosition.parametricOffset, laneSegment.laneInterval));
    }
  }
  return resultPoints;
}

physics::Distance signedDistanceToLane(lane::LaneId const &checkLaneId,
                                       FullRoute const &route,
                                       match::MapMatchedPositionConfidenceList const &mapMatchedPositions)
{
  physics::Distance distance = match::signedDistanceToLane(checkLaneId, mapMatchedPositions);

  auto laneInRoute = findWaypoint(checkLaneId, route);

  if (!laneInRoute.isValid())
  {
    throw std::runtime_error("::ad::map::route::signedDistanceToLane() laneId not found in route");
  }

  if (isRouteDirectionNegative(laneInRoute.laneSegmentIterator->laneInterval))
  {
    distance = distance * -1.;
  }

  return distance;
}

physics::Distance calcLength(FullRoute const &fullRoute)
{
  physics::Distance result(0.);
  for (auto const &roadSegment : fullRoute.roadSegments)
  {
    result += calcLength(roadSegment);
  }
  return result;
}

physics::Distance calcLength(RoadSegment const &roadSegment)
{
  physics::Distance result = std::numeric_limits<physics::Distance>::max();
  for (auto const &laneSegment : roadSegment.drivableLaneSegments)
  {
    physics::Distance laneSegmentLength = calcLength(laneSegment);
    if (laneSegmentLength < result)
    {
      result = laneSegmentLength;
    }
  }
  return result;
}

physics::Distance calcLength(ConnectingRoute const &connectingRoute)
{
  physics::Distance result;
  if (connectingRoute.type == ConnectingRouteType::Invalid)
  {
    result = physics::Distance::getMax();
  }
  else
  {
    result = std::max(calcLength(connectingRoute.routeA), calcLength(connectingRoute.routeB));
  }
  return result;
}

physics::Distance calcLength(RouteIterator const &startIterator, RouteIterator const &endIterator)
{
  physics::Distance distance(0.);
  if (startIterator.isValid() && endIterator.isValid()
      && (std::distance(startIterator.roadSegmentIterator, endIterator.roadSegmentIterator) >= 0u))
  {
    for (auto roadSegmentIter = startIterator.roadSegmentIterator; roadSegmentIter < endIterator.roadSegmentIterator;
         roadSegmentIter++)
    {
      distance += calcLength(*roadSegmentIter);
    }
    distance += calcLength(*endIterator.roadSegmentIterator);
  }
  return distance;
}

physics::Distance
calcLength(RouteParaPoint const &startRouteParaPoint, RouteParaPoint const &endRouteParaPoint, FullRoute const &route)
{
  physics::Distance distance = std::numeric_limits<physics::Distance>::max();

  const auto laneParaPointsStart = getLaneParaPoints(startRouteParaPoint, route);
  const auto laneParaPointsEnd = getLaneParaPoints(endRouteParaPoint, route);

  for (auto startLaneParaPoint : laneParaPointsStart)
  {
    auto startWaypoint = findWaypoint(startLaneParaPoint, route);
    if (startWaypoint.isValid())
    {
      for (auto endLaneParaPoint : laneParaPointsEnd)
      {
        auto endWaypoint = findWaypoint(endLaneParaPoint, route);
        if (endWaypoint.isValid())
        {
          physics::Distance startDistance = calcLength(startWaypoint);
          physics::Distance endDistance = calcLength(endWaypoint);

          distance = std::min(std::fabs(startDistance - endDistance), distance);
        }
      }
    }
  }

  return distance;
}

physics::Duration calcDuration(FullRoute const &fullRoute)
{
  physics::Duration result(0.);
  for (auto const &roadSegment : fullRoute.roadSegments)
  {
    result += calcDuration(roadSegment);
  }
  return result;
}

physics::Duration calcDuration(RoadSegment const &roadSegment)
{
  physics::Duration result = std::numeric_limits<physics::Duration>::max();
  for (auto const &laneSegment : roadSegment.drivableLaneSegments)
  {
    physics::Duration const laneSegmentDuration = calcDuration(laneSegment);
    if (laneSegmentDuration < result)
    {
      result = laneSegmentDuration;
    }
  }
  return result;
}

physics::Duration calcDuration(ConnectingRoute const &connectingRoute)
{
  physics::Duration result = std::max(calcDuration(connectingRoute.routeA), calcDuration(connectingRoute.routeB));
  return result;
}

restriction::SpeedLimitList getSpeedLimits(RoadSegment const &roadSegment)
{
  restriction::SpeedLimitList resultLimits;
  for (auto const &laneSegment : roadSegment.drivableLaneSegments)
  {
    auto const segmentSpeedLimits = getSpeedLimits(laneSegment);
    resultLimits.insert(resultLimits.end(), segmentSpeedLimits.begin(), segmentSpeedLimits.end());
  }
  return resultLimits;
}

restriction::SpeedLimitList getSpeedLimits(LaneSegment const &laneSegment)
{
  return getSpeedLimits(laneSegment.laneInterval);
}

restriction::SpeedLimitList getSpeedLimits(RouteIterator const &startIterator, RouteIterator const &endIterator)
{
  restriction::SpeedLimitList resultLimits;
  if (startIterator.isValid() && endIterator.isValid()
      && (std::distance(startIterator.roadSegmentIterator, endIterator.roadSegmentIterator) >= 0u))
  {
    for (auto roadSegmentIter = startIterator.roadSegmentIterator; roadSegmentIter < endIterator.roadSegmentIterator;
         roadSegmentIter++)
    {
      auto const segmentSpeedLimits = getSpeedLimits(*roadSegmentIter);
      resultLimits.insert(resultLimits.end(), segmentSpeedLimits.begin(), segmentSpeedLimits.end());
    }
  }
  return resultLimits;
}

restriction::SpeedLimitList getSpeedLimits(FullRoute const &fullRoute)
{
  restriction::SpeedLimitList resultLimits;
  for (auto const &roadSegment : fullRoute.roadSegments)
  {
    auto const segmentSpeedLimits = getSpeedLimits(roadSegment);
    resultLimits.insert(resultLimits.end(), segmentSpeedLimits.begin(), segmentSpeedLimits.end());
  }
  return resultLimits;
}

restriction::SpeedLimitList getSpeedLimits(ConnectingRoute const &connectingRoute)
{
  restriction::SpeedLimitList resultLimits = getSpeedLimits(connectingRoute.routeA);
  auto const speedLimitsB = getSpeedLimits(connectingRoute.routeA);
  resultLimits.insert(resultLimits.end(), speedLimitsB.begin(), speedLimitsB.end());
  return resultLimits;
}

bool isWithinInterval(RoadSegment const &roadSegment, point::ParaPoint const &point)
{
  for (auto const &laneSegment : roadSegment.drivableLaneSegments)
  {
    if (isWithinInterval(laneSegment.laneInterval, point))
    {
      return true;
    }
  }

  return false;
}

void clearLaneSegmentPredecessors(RoadSegment &roadSegment)
{
  for (auto &laneSegment : roadSegment.drivableLaneSegments)
  {
    laneSegment.predecessors.clear();
  }
}

void clearLaneSegmentSuccessors(RoadSegment &roadSegment)
{
  for (auto &laneSegment : roadSegment.drivableLaneSegments)
  {
    laneSegment.successors.clear();
  }
}

void updateLaneSegmentNeighbors(RoadSegment &roadSegment)
{
  if (roadSegment.drivableLaneSegments.empty())
  {
    return;
  }

  roadSegment.drivableLaneSegments.front().leftNeighbor = lane::LaneId();
  roadSegment.drivableLaneSegments.back().rightNeighbor = lane::LaneId();

  auto leftLaneSegmentIter = roadSegment.drivableLaneSegments.begin();
  auto rightLaneSegmentIter = leftLaneSegmentIter;
  rightLaneSegmentIter++;

  while (rightLaneSegmentIter != std::end(roadSegment.drivableLaneSegments))
  {
    leftLaneSegmentIter->rightNeighbor = rightLaneSegmentIter->laneInterval.laneId;
    rightLaneSegmentIter->leftNeighbor = leftLaneSegmentIter->laneInterval.laneId;

    leftLaneSegmentIter = rightLaneSegmentIter;
    rightLaneSegmentIter++;
  }
}

void updateLaneSegmentSuccessors(RoadSegment &roadSegment, RoadSegment const &successorSegment)
{
  for (auto &laneSegment : roadSegment.drivableLaneSegments)
  {
    laneSegment.successors.erase(std::remove_if(std::begin(laneSegment.successors),
                                                std::end(laneSegment.successors),
                                                [&successorSegment](lane::LaneId const &successorId) {
                                                  auto findResult
                                                    = std::find_if(std::begin(successorSegment.drivableLaneSegments),
                                                                   std::end(successorSegment.drivableLaneSegments),
                                                                   [&successorId](LaneSegment const &segment) {
                                                                     return segment.laneInterval.laneId == successorId;
                                                                   });
                                                  return findResult == std::end(successorSegment.drivableLaneSegments);
                                                }),
                                 laneSegment.successors.end());
  }
}

void updateLaneSegmentPredecessors(RoadSegment &roadSegment, RoadSegment const &predecessorSegment)
{
  for (auto &laneSegment : roadSegment.drivableLaneSegments)
  {
    laneSegment.predecessors.erase(
      std::remove_if(std::begin(laneSegment.predecessors),
                     std::end(laneSegment.predecessors),
                     [&predecessorSegment](lane::LaneId const &predecessorId) {
                       auto findResult = std::find_if(std::begin(predecessorSegment.drivableLaneSegments),
                                                      std::end(predecessorSegment.drivableLaneSegments),
                                                      [&predecessorId](LaneSegment const &segment) {
                                                        return segment.laneInterval.laneId == predecessorId;
                                                      });
                       return findResult == std::end(predecessorSegment.drivableLaneSegments);
                     }),
      laneSegment.predecessors.end());
  }
}

void updateLaneConnections(FullRoute &fullRoute)
{
  if (fullRoute.roadSegments.empty())
  {
    return;
  }

  clearLaneSegmentPredecessors(fullRoute.roadSegments.front());
  clearLaneSegmentSuccessors(fullRoute.roadSegments.back());
  updateLaneSegmentNeighbors(fullRoute.roadSegments.front());

  auto previousSegment = fullRoute.roadSegments.begin();
  auto nextSegment = previousSegment;
  nextSegment++;

  while (nextSegment != std::end(fullRoute.roadSegments))
  {
    updateLaneSegmentSuccessors(*previousSegment, *nextSegment);
    updateLaneSegmentPredecessors(*nextSegment, *previousSegment);
    updateLaneSegmentNeighbors(*nextSegment);

    previousSegment = nextSegment;
    nextSegment++;
  }
}

physics::Distance calcLength(FindWaypointResult const &findWaypointResult)
{
  physics::Distance result(0.);
  if (findWaypointResult.isValid())
  {
    auto roadSegmentIter = findWaypointResult.queryRoute.roadSegments.begin();
    // all segments before
    for (; roadSegmentIter != findWaypointResult.queryRoute.roadSegments.end()
         && roadSegmentIter != findWaypointResult.roadSegmentIterator;
         ++roadSegmentIter)
    {
      result += calcLength(*roadSegmentIter);
    }

    // the result segment interval
    if (roadSegmentIter == findWaypointResult.roadSegmentIterator)
    {
      // ensure that the interval is actually within the range
      auto laneSegmentIter = roadSegmentIter->drivableLaneSegments.begin();
      while (laneSegmentIter != roadSegmentIter->drivableLaneSegments.end()
             && laneSegmentIter != findWaypointResult.laneSegmentIterator)
      {
        ++laneSegmentIter;
      }

      if (laneSegmentIter == findWaypointResult.laneSegmentIterator)
      {
        LaneInterval calcInterval = laneSegmentIter->laneInterval;
        calcInterval.end = findWaypointResult.queryPosition.parametricOffset;
        result += calcLength(calcInterval);
      }
      else
      {
        throw(std::runtime_error(
          "::ad::map::route::calcLength(FindWaypointResult) intervalIter of the result is not valid"));
      }
    }
    else
    {
      throw(std::runtime_error(
        "::ad::map::route::calcLength(FindWaypointResult) roadSegmentIterator of the result is not valid"));
    }
  }
  return result;
}

FindWaypointResult::FindWaypointResult(FullRoute const &route)
  : queryRoute(route)
  , roadSegmentIterator(queryRoute.roadSegments.end())
{
}

FindWaypointResult &FindWaypointResult::operator=(FindWaypointResult const &other)
{
  if ((&this->queryRoute) != (&other.queryRoute))
  {
    throw std::invalid_argument("FindWaypointResult::operator= incompatible input parameter");
  }
  if (this != &other)
  {
    this->queryPosition = other.queryPosition;
    this->roadSegmentIterator = other.roadSegmentIterator;
    this->laneSegmentIterator = other.laneSegmentIterator;
  }
  return *this;
}

FindWaypointResult FindWaypointResult::getLeftLane() const
{
  FindWaypointResult result(queryRoute);
  if (!isValid() || (!lane::isValid(laneSegmentIterator->leftNeighbor, false)))
  {
    return result;
  }

  result = *this;
  result.laneSegmentIterator++;
  if (!result.isValid() || (result.laneSegmentIterator->laneInterval.laneId != laneSegmentIterator->leftNeighbor))
  {
    throw std::runtime_error("ad::map::route::FindWaypointResult::getLeftLane()>> Route inconsistent: "
                             "left lane not found");
  }
  result.queryPosition.laneId = result.laneSegmentIterator->laneInterval.laneId;
  return result;
}

FindWaypointResult FindWaypointResult::getRightLane() const
{
  FindWaypointResult result(queryRoute);
  if (!isValid() || (!lane::isValid(laneSegmentIterator->rightNeighbor, false)))
  {
    return result;
  }

  result = *this;
  result.laneSegmentIterator--;
  if (!result.isValid() || (result.laneSegmentIterator->laneInterval.laneId != laneSegmentIterator->rightNeighbor))
  {
    throw std::runtime_error("ad::map::route::FindWaypointResult::getRightLane()>> Route inconsistent: "
                             "right lane not found");
  }
  result.queryPosition.laneId = result.laneSegmentIterator->laneInterval.laneId;
  return result;
}

// supporting function used commonly by getSuccessorLanes() and getPredecessorLanes()
std::vector<FindWaypointResult> getLanesOfCurrentRoadSegment(FindWaypointResult &result,
                                                             lane::LaneIdList const &expectedLanesVector)
{
  std::vector<FindWaypointResult> resultList;

  std::set<lane::LaneId> expectedLanes;
  expectedLanes.insert(expectedLanesVector.begin(), expectedLanesVector.end());
  for (result.laneSegmentIterator = result.roadSegmentIterator->drivableLaneSegments.begin();
       result.laneSegmentIterator != result.roadSegmentIterator->drivableLaneSegments.end();
       result.laneSegmentIterator++)
  {
    auto const findResult = expectedLanes.find(result.laneSegmentIterator->laneInterval.laneId);
    if (findResult != expectedLanes.end())
    {
      // found a predecessor
      if (!result.isValid())
      {
        throw std::runtime_error(
          "ad::map::route::FindWaypointResult::getLanesOfCurrentRoadSegment()>> unexpected error");
      }
      result.queryPosition.laneId = result.laneSegmentIterator->laneInterval.laneId;
      resultList.push_back(result);
      expectedLanes.erase(findResult);
    }
  }

  if (!expectedLanes.empty())
  {
    throw std::runtime_error("ad::map::route::FindWaypointResult::getLanesOfCurrentRoadSegment()>> Route inconsistent: "
                             "not all expected lanes found within current road segment");
  }
  return resultList;
}

std::vector<FindWaypointResult> FindWaypointResult::getSuccessorLanes() const
{
  std::vector<FindWaypointResult> resultList;
  if (!isValid() || (laneSegmentIterator->successors.size() == 0u))
  {
    return resultList;
  }

  FindWaypointResult result(queryRoute);
  result = *this;
  result.roadSegmentIterator++;
  if (result.roadSegmentIterator == result.queryRoute.roadSegments.end())
  {
    throw std::runtime_error("ad::map::route::FindWaypointResult::getSuccessorLanes()>> Route inconsistent: "
                             "next road segment not found");
  }
  return getLanesOfCurrentRoadSegment(result, laneSegmentIterator->successors);
}

std::vector<FindWaypointResult> FindWaypointResult::getPredecessorLanes() const
{
  if (!isValid() || (laneSegmentIterator->predecessors.size() == 0u))
  {
    return std::vector<FindWaypointResult>();
  }

  FindWaypointResult result(queryRoute);
  result = *this;
  if (result.roadSegmentIterator == result.queryRoute.roadSegments.begin())
  {
    throw std::runtime_error("ad::map::route::FindWaypointResult::getPredecessorLanes()>> Route inconsistent: "
                             "previous road segment not found");
  }
  result.roadSegmentIterator--;
  return getLanesOfCurrentRoadSegment(result, laneSegmentIterator->predecessors);
}

FindWaypointResult
findWaypointImpl(point::ParaPoint const &position, route::FullRoute const &route, bool considerRouteStart)
{
  for (route::RoadSegmentList::const_iterator roadSegmentIter = route.roadSegments.begin();
       roadSegmentIter != route.roadSegments.end();
       ++roadSegmentIter)
  {
    for (route::LaneSegmentList::const_iterator laneSegmentIter = roadSegmentIter->drivableLaneSegments.begin();
         laneSegmentIter != roadSegmentIter->drivableLaneSegments.end();
         ++laneSegmentIter)
    {
      if (position.laneId == laneSegmentIter->laneInterval.laneId)
      {
        if (!considerRouteStart)
        {
          point::ParaPoint startPoint;
          startPoint.laneId = position.laneId;
          if (isRouteDirectionPositive(laneSegmentIter->laneInterval))
          {
            startPoint.parametricOffset = laneSegmentIter->laneInterval.start;
          }
          else
          {
            startPoint.parametricOffset = laneSegmentIter->laneInterval.end;
          }
          return FindWaypointResult(route, startPoint, roadSegmentIter, laneSegmentIter);
        }
        else
        {
          // due to numeric inaccuracies we need to check not just the internal itself
          // but also some surroundings
          // @todo: Ideally this has to be covered by the laneInterval operations
          // For now, just extend the Interval by 0.5 meter on each end
          LaneInterval laneInterval = laneSegmentIter->laneInterval;
          laneInterval = extendIntervalFromStart(laneInterval, physics::Distance(0.5));
          laneInterval = extendIntervalFromEnd(laneInterval, physics::Distance(0.5));

          if (isWithinInterval(laneInterval, position))
          {
            return FindWaypointResult(route, position, roadSegmentIter, laneSegmentIter);
          }
        }
      }
    }
  }
  return FindWaypointResult(route);
}

FindWaypointResult findWaypoint(point::ParaPoint const &position, route::FullRoute const &route)
{
  return findWaypointImpl(position, route, true);
}

FindWaypointResult findWaypoint(lane::LaneId const &laneId, route::FullRoute const &route)
{
  point::ParaPoint findPoint;
  findPoint.laneId = laneId;
  findPoint.parametricOffset = physics::ParametricValue(0.5);
  return findWaypointImpl(findPoint, route, false);
}

FindWaypointResult findNearestWaypoint(point::ParaPointList const &positions, route::FullRoute const &route)
{
  FindWaypointResult resultWaypoint(route);

  for (auto const &position : positions)
  {
    auto findResult = findWaypoint(position, route);
    if (findResult.isValid())
    {
      if ( // no other result yet
        (!resultWaypoint.isValid())
        // new result is nearer already on route segment level
        || (findResult.roadSegmentIterator < resultWaypoint.roadSegmentIterator))
      {
        resultWaypoint = findResult;
      }
      else if (findResult.roadSegmentIterator == resultWaypoint.roadSegmentIterator)
      {
        // new result is on the same segment, so let the parametricOffset value of the query decide
        if (isRouteDirectionPositive(resultWaypoint.laneSegmentIterator->laneInterval))
        {
          if (findResult.queryPosition.parametricOffset < resultWaypoint.queryPosition.parametricOffset)
          {
            resultWaypoint = findResult;
          }
        }
        else
        {
          if (findResult.queryPosition.parametricOffset > resultWaypoint.queryPosition.parametricOffset)
          {
            resultWaypoint = findResult;
          }
        }
      }
    }
  }

  return resultWaypoint;
}

FindWaypointResult findNearestWaypoint(match::MapMatchedPositionConfidenceList const &mapMatchedPositions,
                                       route::FullRoute const &route)
{
  return findNearestWaypoint(match::getParaPoints(mapMatchedPositions), route);
}

FindWaypointResult findCenterWaypoint(match::Object const &object, route::FullRoute const &route)
{
  FindWaypointResult resultWaypoint(route);
  // first try to search the center point as main reference of the object
  if (object.mapMatchedBoundingBox.referencePointPositions.size()
      > static_cast<std::size_t>(match::ObjectReferencePoints::Center))
  {
    resultWaypoint
      = findNearestWaypoint(object.mapMatchedBoundingBox
                              .referencePointPositions[static_cast<std::size_t>(match::ObjectReferencePoints::Center)],
                            route);
  }
  if (!resultWaypoint.isValid())
  {
    // sort the occupied region center points by the length of the region
    // to start the search with the largest longitudinal region (which is most likely the main lane)
    std::multimap<physics::Distance, point::ParaPoint, std::greater<physics::Distance>> occupiedRegionCenters;
    for (auto const &occupiedRegion : object.mapMatchedBoundingBox.laneOccupiedRegions)
    {
      auto const occupiedRegionLength = lane::calcLength(occupiedRegion);
      auto const occupiedRegionCenter = match::getCenterParaPoint(occupiedRegion);
      occupiedRegionCenters.insert({occupiedRegionLength, occupiedRegionCenter});
    }
    for (auto const &occupiedRegionCenter : occupiedRegionCenters)
    {
      resultWaypoint = findWaypoint(occupiedRegionCenter.second, route);
      if (resultWaypoint.isValid())
      {
        break;
      }
    }
  }
  if (!resultWaypoint.isValid())
  {
    // still nothing found, so we take the nearest of the rest of the reference point positions
    point::ParaPointList mapMatchedParaPoints;
    for (auto i = 0u; i < object.mapMatchedBoundingBox.referencePointPositions.size(); ++i)
    {
      if (i != static_cast<std::size_t>(match::ObjectReferencePoints::Center))
      {
        auto paraPoints = match::getParaPoints(object.mapMatchedBoundingBox.referencePointPositions[i]);
        mapMatchedParaPoints.insert(mapMatchedParaPoints.end(), paraPoints.begin(), paraPoints.end());
      }
    }
    resultWaypoint = findNearestWaypoint(mapMatchedParaPoints, route);
  }
  if (!resultWaypoint.isValid())
  {
    // still nothing found, so search if any point of the route is within our occupied regions
    for (auto const &occupiedRegion : object.mapMatchedBoundingBox.laneOccupiedRegions)
    {
      auto laneWaypoint = findWaypoint(occupiedRegion.laneId, route);
      if (laneWaypoint.isValid())
      {
        auto const routeParmetricLaneRange = route::toParametricRange(laneWaypoint.laneSegmentIterator->laneInterval);
        auto const intersectingRange
          = physics::getIntersectionRange(routeParmetricLaneRange, occupiedRegion.longitudinalRange);
        if (physics::isRangeValid(intersectingRange))
        {
          laneWaypoint.queryPosition.parametricOffset = intersectingRange.minimum;
          resultWaypoint = laneWaypoint;
          break;
        }
      }
    }
  }
  return resultWaypoint;
}

FindWaypointResult objectOnRoute(match::MapMatchedObjectBoundingBox const &object, route::FullRoute const &route)
{
  point::ParaPointList positions;
  for (auto const &occupiedRegion : object.laneOccupiedRegions)
  {
    point::ParaPoint point;
    point.laneId = occupiedRegion.laneId;
    point.parametricOffset = occupiedRegion.longitudinalRange.minimum;
    positions.push_back(point);
    point.parametricOffset = occupiedRegion.longitudinalRange.maximum;
    positions.push_back(point);
  }

  return findNearestWaypoint(positions, route);
}

FindWaypointResult intersectionOnRoute(intersection::Intersection const &intersection, route::FullRoute const &route)
{
  FindWaypointResult result(route);

  if (route.roadSegments.empty())
  {
    return result;
  }

  /**
   * Check if we are already inside the intersection
   */
  auto findResult = std::find_if(std::begin(route.roadSegments.front().drivableLaneSegments),
                                 std::end(route.roadSegments.front().drivableLaneSegments),
                                 [&intersection](LaneSegment const &laneSegment) {
                                   return intersection.internalLanes().find(laneSegment.laneInterval.laneId)
                                     != intersection.internalLanes().end();
                                 });
  if (findResult != std::end(route.roadSegments.front().drivableLaneSegments))
  {
    result.laneSegmentIterator = findResult;
    result.roadSegmentIterator = route.roadSegments.begin();
    result.queryPosition.laneId = result.laneSegmentIterator->laneInterval.laneId;
    result.queryPosition.parametricOffset = result.laneSegmentIterator->laneInterval.start;
  }
  else
  {
    result = findNearestWaypoint(intersection.incomingParaPointsOnRoute(), route);
    if (!result.isValid())
    {
      result = findNearestWaypoint(intersection.incomingParaPoints(), route);
    }
  }

  return result;
}

bool isConnectedRoutePartOfAnIntersection(route::ConnectingRoute const &connectingRoute)
{
  return intersection::Intersection::isRoutePartOfAnIntersection(connectingRoute.routeA)
    || intersection::Intersection::isRoutePartOfAnIntersection(connectingRoute.routeB);
}

void alignRouteStartingPoints(point::ParaPoint const &startAlignmentParaPoint, route::FullRoute &route)
{
  if (route.roadSegments.empty())
  {
    return;
  }

  // we need to ensure, that the starting point of all segments are aligned,
  // i.e. the starting points are located on an imaginary line on the curve radius
  auto const startAlignmentLane = lane::getLane(startAlignmentParaPoint.laneId);
  auto const startAlignmentPoint = lane::getProjectedParametricPoint(
    startAlignmentLane, startAlignmentParaPoint.parametricOffset, physics::ParametricValue(0.5));

  for (route::LaneSegmentList::iterator laneSegmentIter = route.roadSegments.front().drivableLaneSegments.begin();
       laneSegmentIter != route.roadSegments.front().drivableLaneSegments.end();
       ++laneSegmentIter)
  {
    if (laneSegmentIter->laneInterval.laneId != startAlignmentParaPoint.laneId)
    {
      auto lane = lane::getLane(laneSegmentIter->laneInterval.laneId);
      auto rightT = point::findNearestPointOnEdge(lane.edgeRight, startAlignmentPoint);
      auto leftT = point::findNearestPointOnEdge(lane.edgeLeft, startAlignmentPoint);
      laneSegmentIter->laneInterval.start = 0.5 * (rightT + leftT);
    }
  }
}

void alignRouteEndingPoints(point::ParaPoint const &alignmentParaPoint, route::FullRoute &route)
{
  if (route.roadSegments.empty())
  {
    return;
  }

  // we need to ensure, that the end point of all segments are aligned,
  // i.e. the end points are located on an imaginary line on the curve radius
  auto const alignmentLane = lane::getLane(alignmentParaPoint.laneId);
  auto const alignmentPoint = lane::getProjectedParametricPoint(
    alignmentLane, alignmentParaPoint.parametricOffset, physics::ParametricValue(0.5));

  for (route::LaneSegmentList::iterator laneSegmentIter = route.roadSegments.back().drivableLaneSegments.begin();
       laneSegmentIter != route.roadSegments.back().drivableLaneSegments.end();
       ++laneSegmentIter)
  {
    if (laneSegmentIter->laneInterval.laneId != alignmentParaPoint.laneId)
    {
      auto lane = lane::getLane(laneSegmentIter->laneInterval.laneId);
      auto rightT = point::findNearestPointOnEdge(lane.edgeRight, alignmentPoint);
      auto leftT = point::findNearestPointOnEdge(lane.edgeLeft, alignmentPoint);
      laneSegmentIter->laneInterval.end = 0.5 * (rightT + leftT);
    }
  }
}

ShortenRouteResult shortenRoute(point::ParaPointList const &currentPositions,
                                route::FullRoute &route,
                                ShortenRouteMode const shortenRouteMode)
{
  if (route.roadSegments.empty())
  {
    return ShortenRouteResult::FailedRouteEmpty;
  }

  auto findWaypointResult = findNearestWaypoint(currentPositions, route);

  if (findWaypointResult.isValid())
  {
    ShortenRouteResult result = ShortenRouteResult::Succeeded;
    if ((shortenRouteMode == ShortenRouteMode::DontCutIntersectionAndPrependIfSucceededBeforeRoute)
        && (intersection::Intersection::isLanePartOfAnIntersection(
             findWaypointResult.laneSegmentIterator->laneInterval.laneId)))
    {
      // check for intersection
      // check if we are right before the route ---> do not clear in this case
      for (auto roadSegmentIter = route.roadSegments.begin(); roadSegmentIter != route.roadSegments.end();
           roadSegmentIter++)
      {
        RouteIterator routeIterator(route, roadSegmentIter);
        route::RoadSegmentList::const_iterator routePreviousSegmentIter;
        auto intersectionStarts
          = intersection::Intersection::isRoadSegmentEnteringIntersection(routeIterator, routePreviousSegmentIter);
        if (intersectionStarts)
        {
          // erase leading route in front of the intersection
          route.roadSegments.erase(route.roadSegments.begin(), routePreviousSegmentIter);

          // and shorten the previous segments lane intervals
          for (route::LaneSegmentList::iterator laneSegmentIter
               = route.roadSegments.front().drivableLaneSegments.begin();
               laneSegmentIter != route.roadSegments.front().drivableLaneSegments.end();
               ++laneSegmentIter)
          {
            laneSegmentIter->laneInterval.start = laneSegmentIter->laneInterval.end;
          }

          // continue loop to search for further intersections
          result = ShortenRouteResult::SucceededIntersectionNotCut;
        }

        if (roadSegmentIter == findWaypointResult.roadSegmentIterator)
        {
          break;
        }
      }
    }

    if (result != ShortenRouteResult::SucceededIntersectionNotCut)
    {
      // erase leading route
      route.roadSegments.erase(route.roadSegments.begin(), findWaypointResult.roadSegmentIterator);

      // in addition, we have to shorten the parametric offsets
      for (route::LaneSegmentList::iterator laneSegmentIter = route.roadSegments.front().drivableLaneSegments.begin();
           laneSegmentIter != route.roadSegments.front().drivableLaneSegments.end();
           ++laneSegmentIter)
      {
        if (isWithinInterval(laneSegmentIter->laneInterval, findWaypointResult.queryPosition.parametricOffset))
        {
          laneSegmentIter->laneInterval.start = findWaypointResult.queryPosition.parametricOffset;
        }
        else if (!isDegenerated(laneSegmentIter->laneInterval)
                 && isAfterInterval(laneSegmentIter->laneInterval, findWaypointResult.queryPosition.parametricOffset))
        {
          laneSegmentIter->laneInterval.start = laneSegmentIter->laneInterval.end;
        }
      }

      if (route::isDegenerated(route.roadSegments.front().drivableLaneSegments.front().laneInterval))
      {
        bool removeDegeneratedRouteStart = true;
        if (shortenRouteMode == ShortenRouteMode::DontCutIntersectionAndPrependIfSucceededBeforeRoute)
        {
          if (route.roadSegments.size() > 1u)
          {
            RouteIterator routeIterator(route, route.roadSegments.begin() + 1);
            route::RoadSegmentList::const_iterator routePreviousSegmentIter;
            auto intersectionStarts
              = intersection::Intersection::isRoadSegmentEnteringIntersection(routeIterator, routePreviousSegmentIter);
            removeDegeneratedRouteStart = !intersectionStarts;
          }
        }

        if (removeDegeneratedRouteStart)
        {
          // remove also degenerated route start segments
          route.roadSegments.erase(route.roadSegments.begin());
        }
        else
        {
          result = ShortenRouteResult::SucceededIntersectionNotCut;
        }
      }
      else
      {
        alignRouteStartingPoints(findWaypointResult.queryPosition, route);
      }
    }

    if (route.roadSegments.empty())
    {
      return ShortenRouteResult::SucceededRouteEmpty;
    }
    else
    {
      // remove predecessors
      clearLaneSegmentPredecessors(route.roadSegments.front());
      return result;
    }
  }

  // check if we are right before the route ---> do not clear in this case
  for (auto &laneSegment : route.roadSegments.front().drivableLaneSegments)
  {
    for (auto const &currentPosition : currentPositions)
    {
      if (isBeforeInterval(laneSegment.laneInterval, currentPosition))
      {
        if (shortenRouteMode == ShortenRouteMode::Normal)
        {
          return ShortenRouteResult::SucceededBeforeRoute;
        }
        else
        {
          laneSegment.laneInterval.start = currentPosition.parametricOffset;
          alignRouteStartingPoints(currentPosition, route);
          return ShortenRouteResult::Succeeded;
        }
      }
    }
  }

  // check if we are right after the route ---> clear in this case, but still return true
  for (auto const &laneSegment : route.roadSegments.back().drivableLaneSegments)
  {
    for (auto const &currentPosition : currentPositions)
    {
      if (!isDegenerated(laneSegment.laneInterval) && isAfterInterval(laneSegment.laneInterval, currentPosition))
      {
        route.roadSegments.clear();
        return ShortenRouteResult::SucceededRouteEmpty;
      }
    }
  }

  // we are neither right before nor right after nor on the route --> clear
  route.roadSegments.clear();
  return ShortenRouteResult::FailedRouteEmpty;
}

ShortenRouteResult
shortenRoute(point::ParaPoint const &currentPosition, route::FullRoute &route, ShortenRouteMode const shortenRouteMode)
{
  point::ParaPointList currentPositions = {currentPosition};
  return shortenRoute(currentPositions, route, shortenRouteMode);
}

ShortenRouteResult shortenRoute(match::MapMatchedPositionConfidenceList const &mapMatchedPositions,
                                route::FullRoute &route,
                                ShortenRouteMode const shortenRouteMode)
{
  return shortenRoute(match::getParaPoints(mapMatchedPositions), route, shortenRouteMode);
}

void shortenRouteToDistance(route::FullRoute &route, const physics::Distance &length)
{
  auto roadSegmentIterator = route.roadSegments.begin();
  auto remainingLength = length;
  while ((roadSegmentIterator != route.roadSegments.end()) && (remainingLength > physics::Distance(0)))
  {
    auto const segmentLength = calcLength(*roadSegmentIterator);
    if (segmentLength <= remainingLength)
    {
      remainingLength -= segmentLength;
      roadSegmentIterator++;
    }
    else
    {
      // this is the last remaining segment; it has to be shortened
      if (intersection::Intersection::isLanePartOfAnIntersection(
            roadSegmentIterator->drivableLaneSegments.front().laneInterval.laneId))
      {
        // don't cut in between of intersections, rather keep the whole intersection within the route
        for (roadSegmentIterator++; (roadSegmentIterator != route.roadSegments.end()); roadSegmentIterator++)
        {
          if (!intersection::Intersection::isLanePartOfAnIntersection(
                roadSegmentIterator->drivableLaneSegments.front().laneInterval.laneId))
          {
            // outside the intersection, so we can cut the rest
            break;
          }
        }
      }
      else
      {
        auto const deltaLength = segmentLength - remainingLength;
        shortenSegmentFromEnd(*roadSegmentIterator, deltaLength);
        // and push the iterator to the next, where we then can cut
        roadSegmentIterator++;
      }
      // nothing remains, loop ends now
      remainingLength = physics::Distance(0);
    }
  }
  route.roadSegments.erase(roadSegmentIterator, route.roadSegments.end());
  if (!route.roadSegments.empty())
  {
    // remove successors from last road segment
    clearLaneSegmentSuccessors(route.roadSegments.back());
  }
}

void removeLastRoadSegment(route::FullRoute &route)
{
  if (!route.roadSegments.empty())
  {
    route.roadSegments.pop_back();
    if (!route.roadSegments.empty())
    {
      // remove successors from last road segment
      clearLaneSegmentSuccessors(route.roadSegments.back());
    }
  }
}

void removeLastRoadSegmentIfDegenerated(route::FullRoute &route)
{
  // first drop degenerated road segments at the end of route to ensure we get proper direction
  if (!route.roadSegments.empty()
      && (route.roadSegments.back().drivableLaneSegments.empty()
          || isDegenerated(route.roadSegments.back().drivableLaneSegments.front().laneInterval)))
  {
    removeLastRoadSegment(route);
  }
}

bool prepareRouteForExtension(route::FullRoute &route,
                              route::planning::RoutingParaPoint &routeExtensionStartPoint,
                              route::LaneSegment &extensionReferenceLaneSegment)
{
  // first drop degenerated road segments at the end of route to ensure we get proper direction
  removeLastRoadSegmentIfDegenerated(route);

  // abort on degenerated route
  if (route.roadSegments.empty() || route.roadSegments.back().drivableLaneSegments.empty())
  {
    return false;
  }

  extensionReferenceLaneSegment = route.roadSegments.back().drivableLaneSegments.front();
  routeExtensionStartPoint = route::planning::createRoutingPoint(
    extensionReferenceLaneSegment.laneInterval.laneId,
    extensionReferenceLaneSegment.laneInterval.start,
    (isRouteDirectionPositive(extensionReferenceLaneSegment.laneInterval) ? planning::RoutingDirection::POSITIVE
                                                                          : planning::RoutingDirection::NEGATIVE));
  return true;
}

route::FullRoute mergeRouteExtension(route::FullRoute const &route,
                                     route::LaneSegment const &extensionReferenceLaneSegment,
                                     route::FullRoute const &routeExtension)
{
  FullRoute newRoute(route);
  removeLastRoadSegment(newRoute);
  for (auto const &roadSegment : routeExtension.roadSegments)
  {
    route::appendRoadSegmentToRoute(roadSegment.drivableLaneSegments.front().laneInterval,
                                    extensionReferenceLaneSegment.routeLaneOffset
                                      + roadSegment.drivableLaneSegments.front().routeLaneOffset,
                                    newRoute);
  }
  planning::updateRoutePlanningCounters(newRoute);
  return newRoute;
}

template <typename T> bool extendRouteToDestinationsT(route::FullRoute &route, const std::vector<T> &dest)
{
  route::planning::RoutingParaPoint routeExtensionStartPoint;
  route::LaneSegment extensionReferenceLaneSegment;
  if (!prepareRouteForExtension(route, routeExtensionStartPoint, extensionReferenceLaneSegment))
  {
    return false;
  }

  auto routeExtension = route::planning::planRoute(routeExtensionStartPoint, dest, route.routeCreationMode);

  route = mergeRouteExtension(route, extensionReferenceLaneSegment, routeExtension);

  return true;
}

bool extendRouteToDestinations(route::FullRoute &route, const std::vector<route::planning::RoutingParaPoint> &dest)
{
  return extendRouteToDestinationsT(route, dest);
}

bool extendRouteToDestinations(route::FullRoute &route, const std::vector<point::GeoPoint> &dest)
{
  return extendRouteToDestinationsT(route, dest);
}

bool extendRouteToDestinations(route::FullRoute &route, const std::vector<point::ENUPoint> &dest)
{
  return extendRouteToDestinationsT(route, dest);
}

bool extendRouteToDistance(route::FullRoute &route,
                           const physics::Distance &length,
                           route::FullRouteList &additionalRoutes)
{
  if (!additionalRoutes.empty())
  {
    access::getLogger()->error("ad::map::route::extendRouteToDistance: additional Routes parameter is not empty, "
                               "containing {} elements. Aborting.",
                               additionalRoutes.size());
    return false;
  }

  // check length of route and abort if long enough
  auto routeLength = route::calcLength(route);
  auto distance = length - routeLength;
  if (distance < physics::Distance(0))
  {
    return true;
  }

  route::planning::RoutingParaPoint routeExtensionStartPoint;
  route::LaneSegment extensionReferenceLaneSegment;
  if (!prepareRouteForExtension(route, routeExtensionStartPoint, extensionReferenceLaneSegment))
  {
    return false;
  }

  distance += route::calcLength(extensionReferenceLaneSegment.laneInterval);

  auto routeExtensions
    = route::planning::predictRoutesOnDistance(routeExtensionStartPoint, distance, route.routeCreationMode);

  auto it = routeExtensions.begin();
  if (it != routeExtensions.end())
  {
    ++it;
  }
  for (; it != routeExtensions.end(); ++it)
  {
    auto const &routeExtension = *it;
    auto newRoute = mergeRouteExtension(route, extensionReferenceLaneSegment, routeExtension);
    additionalRoutes.push_back(newRoute);
  }

  // extend route itself
  if (routeExtensions.size() > 0)
  {
    auto const &routeExtension = routeExtensions.front();
    route = mergeRouteExtension(route, extensionReferenceLaneSegment, routeExtension);
  }

  return true;
}

void shortenSegmentFromBegin(RoadSegment &roadSegment, physics::Distance const &distance)
{
  if (roadSegment.drivableLaneSegments.empty())
  {
    throw std::runtime_error("ad::map::route::shortenSegmentFromBegin>> Route inconsistent: "
                             "route contains no drivableLaneSegments");
  }

  auto shortenedInterval = shortenIntervalFromBegin(roadSegment.drivableLaneSegments.front().laneInterval, distance);
  for (auto &laneSegment : roadSegment.drivableLaneSegments)
  {
    laneSegment.laneInterval.start = shortenedInterval.start;
  }
}

void shortenSegmentFromEnd(RoadSegment &roadSegment, physics::Distance const &distance)
{
  if (roadSegment.drivableLaneSegments.empty())
  {
    throw std::runtime_error("ad::map::route::shortenSegmentFromBegin>> Route inconsistent: "
                             "route contains no drivableLaneSegments");
  }

  auto shortenedInterval = shortenIntervalFromEnd(roadSegment.drivableLaneSegments.front().laneInterval, distance);
  for (auto &laneSegment : roadSegment.drivableLaneSegments)
  {
    laneSegment.laneInterval.end = shortenedInterval.end;
  }
}

bool calculateRouteParaPointAtDistance(route::FullRoute const &route,
                                       route::RouteParaPoint const &origin,
                                       physics::Distance const &distance,
                                       route::RouteParaPoint &resultingPoint)
{
  RouteIterator routeIterator = getRouteIterator(origin, route);

  physics::Distance accumulatedDistance(0.);

  bool routeParaPointFound = false;

  while (!routeParaPointFound && routeIterator.isValid())
  {
    physics::ParametricValue originOffset(0.);
    const physics::Distance segmentLength = calcLength((*routeIterator.roadSegmentIterator));

    physics::Distance distanceToSegmentBorder = segmentLength;
    if (routeIterator.roadSegmentIterator->segmentCountFromDestination == origin.segmentCountFromDestination)
    {
      originOffset = origin.parametricOffset;
      if (distance < physics::Distance(0))
      {
        distanceToSegmentBorder = segmentLength * (origin.parametricOffset);
      }
      else
      {
        distanceToSegmentBorder = segmentLength * (physics::ParametricValue(1.0) - origin.parametricOffset);
      }
    }
    else
    {
      if (distance < physics::Distance(0))
      {
        originOffset = physics::ParametricValue(1.);
      }
    }

    if ((accumulatedDistance + distanceToSegmentBorder) >= std::fabs(distance))
    {
      physics::Distance const remainingLength = std::fabs(distance) - accumulatedDistance;

      resultingPoint.routePlanningCounter = route.routePlanningCounter;
      resultingPoint.segmentCountFromDestination = routeIterator.roadSegmentIterator->segmentCountFromDestination;
      physics::ParametricValue delta(remainingLength / segmentLength);
      if (distance < physics::Distance(0))
      {
        resultingPoint.parametricOffset = originOffset - delta;
      }
      else
      {
        resultingPoint.parametricOffset = originOffset + delta;
      }
      accumulatedDistance = std::fabs(distance);
      routeParaPointFound = true;
    }
    else
    {
      accumulatedDistance += distanceToSegmentBorder;
    }

    if (distance < physics::Distance(0))
    {
      if (routeIterator.roadSegmentIterator != std::begin(routeIterator.route.roadSegments))
      {
        routeIterator.roadSegmentIterator--;
      }
      else
      {
        routeIterator.roadSegmentIterator = std::end(routeIterator.route.roadSegments);
      }
    }
    else
    {
      routeIterator.roadSegmentIterator++;
    }
  }

  return routeParaPointFound;
}

bool getRouteParaPointFromParaPoint(point::ParaPoint const &paraPoint,
                                    FullRoute const &route,
                                    route::RouteParaPoint &routeParaPoint)
{
  auto const waypointResult = route::findWaypoint(paraPoint, route);

  if (!waypointResult.isValid())
  {
    return false;
  }

  routeParaPoint.routePlanningCounter = route.routePlanningCounter;
  routeParaPoint.segmentCountFromDestination = waypointResult.roadSegmentIterator->segmentCountFromDestination;
  routeParaPoint.parametricOffset
    = std::fabs(paraPoint.parametricOffset - waypointResult.laneSegmentIterator->laneInterval.start);

  return true;
}

route::FullRoute getRouteExpandedTo(route::FullRoute const &route, RouteCreationMode const routeCreationMode)
{
  route::FullRoute expandedRoute;
  expandedRoute.routeCreationMode = routeCreationMode;
  for (const auto &roadSegment : route.roadSegments)
  {
    if (!roadSegment.drivableLaneSegments.empty())
    {
      appendRoadSegmentToRoute(roadSegment.drivableLaneSegments.front().laneInterval,
                               roadSegment.drivableLaneSegments.front().routeLaneOffset,
                               expandedRoute);
    }
  }
  planning::updateRoutePlanningCounters(expandedRoute);

  if (!route.roadSegments.empty())
  {
    if (!route.roadSegments.front().drivableLaneSegments.empty())
    {
      point::ParaPoint const alignmentParaPoint
        = getIntervalStart(route.roadSegments.front().drivableLaneSegments.front().laneInterval);
      alignRouteStartingPoints(alignmentParaPoint, expandedRoute);
    }
    if (!route.roadSegments.back().drivableLaneSegments.empty())
    {
      point::ParaPoint const alignmentParaPoint
        = getIntervalEnd(route.roadSegments.back().drivableLaneSegments.front().laneInterval);
      alignRouteEndingPoints(alignmentParaPoint, expandedRoute);
    }
  }

  return expandedRoute;
}

FullRoute getRouteSection(FindWaypointResult const &currentLane,
                          physics::Distance const &distanceFront,
                          physics::Distance const &distanceEnd,
                          FullRoute const &route,
                          RouteSectionCreationMode const routeSectionCreationMode)
{
  FullRoute resultRoute;
  if (!currentLane.isValid() || (&currentLane.queryRoute != &route))
  {
    return resultRoute;
  }
  resultRoute.fullRouteSegmentCount = route.fullRouteSegmentCount;
  resultRoute.routePlanningCounter = route.routePlanningCounter;

  LaneSegment currentLaneSegment = *currentLane.laneSegmentIterator;

  LaneInterval currentStartInterval;
  currentStartInterval.laneId = currentLane.laneSegmentIterator->laneInterval.laneId;
  currentStartInterval.start = currentLane.laneSegmentIterator->laneInterval.start;
  currentStartInterval.end = currentLane.queryPosition.parametricOffset;

  physics::Distance accumulatedDistanceFront = calcLength(currentStartInterval);

  if (accumulatedDistanceFront >= distanceFront)
  {
    currentLaneSegment.laneInterval
      = shortenIntervalFromBegin(currentLaneSegment.laneInterval, accumulatedDistanceFront - distanceFront);
    accumulatedDistanceFront = distanceFront;
  }
  else
  {
    auto currentPredecessors = currentLane.getPredecessorLanes();
    while ( // required distance not yet reached
      (accumulatedDistanceFront < distanceFront)
      // there are still predecessors available
      && (!currentPredecessors.empty()))
    {
      // another road segment will be added
      route::RoadSegment newRoadSegment;
      newRoadSegment.boundingSphere = currentPredecessors[0].roadSegmentIterator->boundingSphere;
      newRoadSegment.segmentCountFromDestination
        = currentPredecessors[0].roadSegmentIterator->segmentCountFromDestination;

      std::vector<FindWaypointResult> nextPredecessors;
      for (auto const &predecessor : currentPredecessors)
      {
        newRoadSegment.drivableLaneSegments.push_back(*predecessor.laneSegmentIterator);
        auto const furtherPredecessors = predecessor.getPredecessorLanes();
        nextPredecessors.insert(
          std::end(nextPredecessors), std::begin(furtherPredecessors), std::end(furtherPredecessors));
      }

      auto currentSegmentLength = calcLength(newRoadSegment);
      if (accumulatedDistanceFront + currentSegmentLength > distanceFront)
      {
        route::shortenSegmentFromBegin(newRoadSegment, accumulatedDistanceFront + currentSegmentLength - distanceFront);
        accumulatedDistanceFront = distanceFront;
      }
      else
      {
        accumulatedDistanceFront += currentSegmentLength;
      }

      access::getLogger()->trace("ad::map::route::getRouteSection: prepending road segment {}: {} ({})",
                                 newRoadSegment,
                                 accumulatedDistanceFront,
                                 distanceFront);
      resultRoute.roadSegments.insert(std::begin(resultRoute.roadSegments), newRoadSegment);

      // prepare for next cycle
      currentPredecessors.swap(nextPredecessors);
    }
  }

  LaneInterval currentEndInterval;
  currentEndInterval.laneId = currentLane.laneSegmentIterator->laneInterval.laneId;
  currentEndInterval.start = currentLane.queryPosition.parametricOffset;
  currentEndInterval.end = currentLane.laneSegmentIterator->laneInterval.end;

  physics::Distance accumulatedDistanceEnd = calcLength(currentEndInterval);
  if (accumulatedDistanceEnd >= distanceEnd)
  {
    currentLaneSegment.laneInterval
      = shortenIntervalFromEnd(currentLaneSegment.laneInterval, accumulatedDistanceEnd - distanceEnd);
    accumulatedDistanceEnd = distanceEnd;
  }
  else
  {
    currentLaneSegment.laneInterval.end = currentEndInterval.end;
  }

  // current road segment is added
  route::RoadSegment currentRoadSegment;
  currentRoadSegment.boundingSphere = currentLane.roadSegmentIterator->boundingSphere;
  currentRoadSegment.segmentCountFromDestination = currentLane.roadSegmentIterator->segmentCountFromDestination;
  currentRoadSegment.drivableLaneSegments.push_back(currentLaneSegment);
  access::getLogger()->trace("ad::map::route::getRouteSection: appending current road segment {}: {}({}) -> {}({})",
                             currentRoadSegment,
                             accumulatedDistanceFront,
                             distanceFront,
                             accumulatedDistanceEnd,
                             distanceEnd);
  resultRoute.roadSegments.insert(std::end(resultRoute.roadSegments), currentRoadSegment);

  auto currentSuccessors = currentLane.getSuccessorLanes();
  while ( // required distance not yet reached
    (accumulatedDistanceEnd < distanceEnd)
    // there are still successors available
    && (!currentSuccessors.empty()))
  {
    // another road segment will be added
    route::RoadSegment newRoadSegment;
    newRoadSegment.boundingSphere = currentSuccessors[0].roadSegmentIterator->boundingSphere;
    newRoadSegment.segmentCountFromDestination = currentSuccessors[0].roadSegmentIterator->segmentCountFromDestination;

    std::vector<FindWaypointResult> nextSuccessors;
    for (auto const &successor : currentSuccessors)
    {
      newRoadSegment.drivableLaneSegments.push_back(*successor.laneSegmentIterator);
      auto const furtherSuccessors = successor.getSuccessorLanes();
      nextSuccessors.insert(std::end(nextSuccessors), std::begin(furtherSuccessors), std::end(furtherSuccessors));
    }

    auto currentSegmentLength = calcLength(newRoadSegment);
    if (accumulatedDistanceEnd + currentSegmentLength > distanceEnd)
    {
      route::shortenSegmentFromEnd(newRoadSegment, accumulatedDistanceEnd + currentSegmentLength - distanceEnd);
      accumulatedDistanceEnd = distanceEnd;
    }
    else
    {
      accumulatedDistanceEnd += currentSegmentLength;
    }

    access::getLogger()->trace("ad::map::route::getRouteSection: appending road segment {}: {} ({})",
                               newRoadSegment,
                               accumulatedDistanceEnd,
                               distanceEnd);
    resultRoute.roadSegments.insert(std::end(resultRoute.roadSegments), newRoadSegment);

    // prepare for next cycle
    currentSuccessors.swap(nextSuccessors);
  }

  access::getLogger()->trace("ad::map::route::getRouteSection: result before update lane connections {}", resultRoute);
  updateLaneConnections(resultRoute);

  if (routeSectionCreationMode == RouteSectionCreationMode::AllRouteLanes)
  {
    resultRoute = getRouteExpandedTo(resultRoute, route.routeCreationMode);
  }

  access::getLogger()->debug("ad::map::route::getRouteSection({} < {}:{} > {} ) {}",
                             distanceFront,
                             currentLane.laneSegmentIterator->laneInterval.laneId,
                             currentLane.queryPosition.parametricOffset,
                             distanceEnd,
                             resultRoute);

  return resultRoute;
}

FullRoute getRouteSection(point::ParaPoint const &centerPoint,
                          physics::Distance const &distanceFront,
                          physics::Distance const &distanceEnd,
                          FullRoute const &route,
                          RouteSectionCreationMode const routeSectionCreationMode)
{
  return getRouteSection(findWaypoint(centerPoint, route), distanceFront, distanceEnd, route, routeSectionCreationMode);
}

FullRoute getRouteSection(match::Object const &object,
                          FullRoute const &route,
                          RouteSectionCreationMode const routeSectionCreationMode)
{
  auto const objectCenterPointOnRoute = findCenterWaypoint(object, route);
  auto const routeAroundVehicle = getRouteSection(objectCenterPointOnRoute,
                                                  object.enuPosition.dimension.length,
                                                  object.enuPosition.dimension.length,
                                                  route,
                                                  routeSectionCreationMode);
  return routeAroundVehicle;
}

FindLaneChangeResult::FindLaneChangeResult(FullRoute const &route)
  : queryRoute(route)
  , laneChangeStartRouteIterator(queryRoute.roadSegments.end())
  , laneChangeEndRouteIterator(queryRoute.roadSegments.end())
  , laneChangeDirection(LaneChangeDirection::Invalid)
{
}

physics::Distance FindLaneChangeResult::calcZoneLength() const
{
  physics::Distance distance(0.);
  if (isValid())
  {
    for (auto roadSegmentIter = laneChangeStartRouteIterator; roadSegmentIter < laneChangeEndRouteIterator;
         roadSegmentIter++)
    {
      distance += calcLength(*roadSegmentIter);
    }
    distance += calcLength(*laneChangeEndRouteIterator);
  }
  return distance;
}

FindLaneChangeResult findFirstLaneChange(match::MapMatchedPosition const &currentPosition,
                                         route::FullRoute const &route)
{
  // this is what we want to return. Two indices defining the start and the end of the lane change
  FindLaneChangeResult result(route);

  // first, find the iterator where the first lane change needs to be finished at the latest. If the target lane is
  // always blocked the user of this function may want to set a stop line at the routeIteratorLaneChangeEnd. To do
  // this,
  // - we travel along the successors starting from the current position as long as we can
  // - if the next route section is not reachable by successors and is not the end of the route, we found the lane
  //   change end and traverse along the route in the other direction to find the lane change start
  // - if we can travel along the whole route without changing lanes, we return an invalid result (since there is no
  //   lane change on this route segment)
  FindWaypointResult const findWaypointResult = findWaypoint(currentPosition.lanePoint.paraPoint, route);

  if (!findWaypointResult.isValid())
  {
    if (!route.roadSegments.empty())
    {
      access::getLogger()->error(
        "ad::map::route::findFirstLaneChange: Current position is not part of the route {} {}", currentPosition, route);
    }
    // returning an invalid result (as if no lane change was found)
    return result;
  }

  // find the potential end of the lane change
  // if there is no successor, the lane change has to end
  // and if there is more than one successor, the lane change has to end, too
  auto laneChangeEndResult = findWaypointResult;
  for (auto successorLanes = findWaypointResult.getSuccessorLanes(); successorLanes.size() == 1u;
       successorLanes = successorLanes[0].getSuccessorLanes())
  {
    laneChangeEndResult = successorLanes[0];
  }

  // Find the transition lane, to which we need to change to be able to get to the next road segment.
  // The closest neighbor lane to laneChangeEndResult having any successors
  FindWaypointResult rightNeighbor(route);
  std::size_t rightNeighborDistance = 0u;
  for (auto neighbor = laneChangeEndResult.getRightLane(); neighbor.isValid(); neighbor = neighbor.getRightLane())
  {
    rightNeighborDistance++;
    if (!neighbor.getSuccessorLanes().empty())
    {
      rightNeighbor = neighbor;
      break;
    }
  }

  FindWaypointResult leftNeighbor(route);
  std::size_t leftNeighborDistance = 0u;
  for (auto neighbor = laneChangeEndResult.getLeftLane(); neighbor.isValid(); neighbor = neighbor.getLeftLane())
  {
    leftNeighborDistance++;
    if (!neighbor.getSuccessorLanes().empty())
    {
      leftNeighbor = neighbor;
      break;
    }
  }

  FindWaypointResult transitionEndLane(route);
  if (leftNeighbor.isValid() && rightNeighbor.isValid())
  {
    if (leftNeighborDistance < rightNeighborDistance)
    {
      transitionEndLane = leftNeighbor;
      result.laneChangeDirection = LaneChangeDirection::RightToLeft;
    }
    else
    {
      transitionEndLane = rightNeighbor;
      result.laneChangeDirection = LaneChangeDirection::LeftToRight;
    }
  }
  else if (leftNeighbor.isValid())
  {
    transitionEndLane = leftNeighbor;
    result.laneChangeDirection = LaneChangeDirection::RightToLeft;
  }
  else if (rightNeighbor.isValid())
  {
    transitionEndLane = rightNeighbor;
    result.laneChangeDirection = LaneChangeDirection::LeftToRight;
  }
  else
  {
    // there is no lane change at all, since no neighbor has any successor
    access::getLogger()->trace("ad::map::route::no lane change required {} {}", currentPosition, route);
    return result;
  }

  // we found an actual lane change
  result.laneChangeEndRouteIterator = transitionEndLane.roadSegmentIterator;
  result.laneChangeEndLaneSegmentIterator = transitionEndLane.laneSegmentIterator;

  // now, traverse back to the beginning (currentPosiion) from the transition lane
  // if there is no predecessor, the lane change has to begin latest there
  // and if there is more than one predecessor, the lane change has to begin there, too
  bool currentPositionReached = false;

  while (!currentPositionReached)
  {
    auto laneChangeBeginResult = transitionEndLane;
    for (auto predecessorLanes = transitionEndLane.getPredecessorLanes(); predecessorLanes.size() == 1u;
         predecessorLanes = predecessorLanes[0].getPredecessorLanes())
    {
      laneChangeBeginResult = predecessorLanes[0];
    }

    FindWaypointResult transitionStartLane(route);
    if (result.laneChangeDirection == LaneChangeDirection::LeftToRight)
    {
      transitionStartLane = laneChangeBeginResult.getLeftLane();
    }
    else
    {
      transitionStartLane = laneChangeBeginResult.getRightLane();
    }
    if (!transitionStartLane.isValid())
    {
      access::getLogger()->error("ad::map::route::findFirstLaneChange: cannot find valid transition start lane at lane "
                                 "change, begin: {} with lane change direction {} and route: {}",
                                 laneChangeBeginResult.laneSegmentIterator->laneInterval.laneId,
                                 result.laneChangeDirection,
                                 route);

      // returning an invalid result (as if no lane change was found)
      return result;
    }

    result.laneChangeStartRouteIterator = transitionStartLane.roadSegmentIterator;
    result.laneChangeStartLaneSegmentIterator = transitionStartLane.laneSegmentIterator;

    access::getLogger()->debug("ad::map::route::findFirstLaneChange: found valid lane change {} starting at {}  "
                               "laneId[] {} ending at {}  laneId[] {} input position {} and route {}",
                               result.laneChangeDirection,
                               *result.laneChangeStartRouteIterator,
                               result.laneChangeStartLaneSegmentIterator->laneInterval.laneId,
                               *result.laneChangeEndRouteIterator,
                               result.laneChangeEndLaneSegmentIterator->laneInterval.laneId,
                               currentPosition,
                               route);

    if (result.laneChangeStartLaneSegmentIterator->laneInterval.laneId == currentPosition.lanePoint.paraPoint.laneId)
    {
      currentPositionReached = true;
    }
    else
    {
      result.laneChangeEndRouteIterator = result.laneChangeStartRouteIterator;
      result.laneChangeEndLaneSegmentIterator = result.laneChangeStartLaneSegmentIterator;
      transitionEndLane.laneSegmentIterator = result.laneChangeEndLaneSegmentIterator;
      transitionEndLane.roadSegmentIterator = result.laneChangeStartRouteIterator;
    }

    result.numberOfConnectedLaneChanges++;
  }

  return result;
}

void addLaneIdUnique(lane::LaneIdList &laneIds, lane::LaneId const laneId)
{
  auto findResult = std::find(std::begin(laneIds), std::end(laneIds), laneId);
  if (findResult == std::end(laneIds))
  {
    laneIds.push_back(laneId);
  }
}

void addRoutePredecessorRelationToNewSegment(lane::Lane const &lane,
                                             route::LaneSegment &laneSegment,
                                             route::FullRoute &route,
                                             bool routeDirectionPositive)
{
  if (route.roadSegments.empty())
  {
    return;
  }
  for (auto predecessorLaneContact : lane::getContactLanes(
         lane, routeDirectionPositive ? lane::ContactLocation::PREDECESSOR : lane::ContactLocation::SUCCESSOR))
  {
    auto predecessorLaneId = predecessorLaneContact.toLane;
    auto predecessorLaneSegmentIter = std::find_if(std::begin(route.roadSegments.back().drivableLaneSegments),
                                                   std::end(route.roadSegments.back().drivableLaneSegments),
                                                   [&predecessorLaneId](route::LaneSegment const &innerLaneSegment) {
                                                     return innerLaneSegment.laneInterval.laneId == predecessorLaneId;
                                                   });

    if (predecessorLaneSegmentIter != std::end(route.roadSegments.back().drivableLaneSegments))
    {
      addLaneIdUnique(predecessorLaneSegmentIter->successors, laneSegment.laneInterval.laneId);
      addLaneIdUnique(laneSegment.predecessors, predecessorLaneId);
      laneSegment.routeLaneOffset = predecessorLaneSegmentIter->routeLaneOffset;
    }
  }
}

bool isRightNeighbor(lane::ContactLocation const contactLocation, bool const isRouteDirectionPositive)
{
  bool rightNeighbor;
  // determine the neighborhood relationship in respect to route direction
  if (contactLocation == lane::ContactLocation::RIGHT)
  {
    rightNeighbor = isRouteDirectionPositive;
  }
  else // contactLocation == core::ContactLocation::LEFT
  {
    rightNeighbor = !isRouteDirectionPositive;
  }
  return rightNeighbor;
}

void updateRouteLaneOffsetRange(RouteLaneOffset const &routeLaneOffset, FullRoute &route)
{
  if (routeLaneOffset > route.maxLaneOffset)
  {
    route.maxLaneOffset = routeLaneOffset;
  }
  if (routeLaneOffset < route.minLaneOffset)
  {
    route.minLaneOffset = routeLaneOffset;
  }
}

void updateRouteLaneOffset(bool const rightNeighbor, RouteLaneOffset &routeLaneOffset, FullRoute &route)
{
  if (rightNeighbor)
  {
    routeLaneOffset--;
  }
  else
  {
    routeLaneOffset++;
  }
  updateRouteLaneOffsetRange(routeLaneOffset, route);
}

void appendLaneSegmentToRoute(route::LaneInterval const &laneInterval,
                              route::FullRoute &route,
                              route::SegmentCounter const segmentCountFromDestination)
{
  auto lane = lane::getLane(laneInterval.laneId);

  route::RoadSegment roadSegment;
  roadSegment.boundingSphere = lane.boundingSphere;
  roadSegment.segmentCountFromDestination = segmentCountFromDestination;

  bool const routeDirectionPositive = isRouteDirectionPositive(laneInterval);

  route::LaneSegment laneSegment;
  laneSegment.laneInterval = laneInterval;

  addRoutePredecessorRelationToNewSegment(lane, laneSegment, route, routeDirectionPositive);

  roadSegment.drivableLaneSegments.push_back(laneSegment);
  route.roadSegments.push_back(roadSegment);
}

void appendRoadSegmentToRoute(route::LaneInterval const &laneInterval,
                              route::RouteLaneOffset const &routeLaneOffset,
                              FullRoute &route)
{
  auto lane = lane::getLane(laneInterval.laneId);

  route::RoadSegment roadSegment;
  roadSegment.boundingSphere = lane.boundingSphere;
  // this information will be post processed by updateRoutePlanningCounters()
  roadSegment.segmentCountFromDestination = 0u;

  const bool routeDirectionPositive = isRouteDirectionPositive(laneInterval);

  route::LaneSegment laneSegment;
  laneSegment.laneInterval = laneInterval;
  laneSegment.routeLaneOffset = routeLaneOffset;
  updateRouteLaneOffsetRange(routeLaneOffset, route);
  addRoutePredecessorRelationToNewSegment(lane, laneSegment, route, routeDirectionPositive);
  roadSegment.drivableLaneSegments.push_back(laneSegment);

  lane::LaneDirection const direction = lane.direction;
  for (auto contactLocation : {lane::ContactLocation::LEFT, lane::ContactLocation::RIGHT})
  {
    route::RouteLaneOffset currentRouteLaneOffset = routeLaneOffset;
    bool const rightNeighbor = isRightNeighbor(contactLocation, routeDirectionPositive);
    lane::ContactLaneList contactLanes = lane::getContactLanes(lane, contactLocation);

    lane::LaneIdSet visitedLaneIds;
    visitedLaneIds.insert(laneInterval.laneId);

    // we expect that per map model only one contact lane is possible in one direction
    while (contactLanes.size() == 1u)
    {
      // in some broken map cases we could end in an infinite loop here, due to broken neighborhood relations
      // to avoid this issue, add an early return upon detecting the start lane again
      lane::LaneId otherLaneId = contactLanes.front().toLane;

      auto laneNotVisited = visitedLaneIds.insert(otherLaneId);
      if (!laneNotVisited.second)
      {
        access::getLogger()->warn("ad::map::route::appendRoadSegmentToRoute: Broken neighborhood relations detected "
                                  "for LaneId {}, Skipping expansion.",
                                  laneInterval.laneId);
        contactLanes.clear();
        break;
      }

      auto const &otherLane = lane::getLane(otherLaneId);
      if (((route.routeCreationMode == RouteCreationMode::SameDrivingDirection) && (direction == otherLane.direction))
          || ((route.routeCreationMode == RouteCreationMode::AllRoutableLanes) && (lane::isRouteable(otherLane)))
          || (route.routeCreationMode == RouteCreationMode::AllNeighborLanes))
      {
        route::LaneSegment newLaneSegment;
        newLaneSegment.laneInterval.laneId = otherLaneId;
        newLaneSegment.laneInterval.start = laneInterval.start;
        newLaneSegment.laneInterval.end = laneInterval.end;
        if (direction == otherLane.direction)
        {
          newLaneSegment.laneInterval.wrongWay = laneInterval.wrongWay;
        }
        else
        {
          newLaneSegment.laneInterval.wrongWay = !laneInterval.wrongWay;
        }
        updateRouteLaneOffset(rightNeighbor, currentRouteLaneOffset, route);
        newLaneSegment.routeLaneOffset = currentRouteLaneOffset;

        roadSegment.boundingSphere = roadSegment.boundingSphere + otherLane.boundingSphere;

        addRoutePredecessorRelationToNewSegment(otherLane, newLaneSegment, route, routeDirectionPositive);

        // sorting: right lanes are added at front, left lanes at back
        if (rightNeighbor)
        {
          roadSegment.drivableLaneSegments.front().rightNeighbor = newLaneSegment.laneInterval.laneId;
          newLaneSegment.leftNeighbor = roadSegment.drivableLaneSegments.front().laneInterval.laneId;
          roadSegment.drivableLaneSegments.insert(roadSegment.drivableLaneSegments.begin(), newLaneSegment);
        }
        else
        {
          roadSegment.drivableLaneSegments.back().leftNeighbor = newLaneSegment.laneInterval.laneId;
          newLaneSegment.rightNeighbor = roadSegment.drivableLaneSegments.back().laneInterval.laneId;
          roadSegment.drivableLaneSegments.insert(roadSegment.drivableLaneSegments.end(), newLaneSegment);
        }

        // go aside recursively
        contactLanes = lane::getContactLanes(otherLane, contactLocation);
      }
      else
      {
        contactLanes.clear();
      }
    }
    if (!contactLanes.empty())
    {
      throw std::runtime_error(
        "AdRoadNetworkAdm::fillRoadSegment algorithm is not able to handle multiple left/right contact lanes");
    }
  }
  route.roadSegments.push_back(roadSegment);
}

physics::Distance addOpposingLaneSegmentToRoadSegment(point::ParaPoint const &startpoint,
                                                      physics::Distance const &distance,
                                                      route::RoadSegment &roadSegment,
                                                      route::FullRoute &route)
{
  if (roadSegment.drivableLaneSegments.empty())
  {
    return physics::Distance(-1.);
  }

  route::LaneInterval laneInterval;
  laneInterval.laneId = startpoint.laneId;
  laneInterval.start = startpoint.parametricOffset;
  laneInterval.end = roadSegment.drivableLaneSegments[0].laneInterval.end;

  // the lane segment we want to add is opposing the current road segment
  // for right-hand traffic this means, the lane segment is left of the left-most lane segment
  // for left-hand traffic this is the segment right of the right-most lane segment
  LaneSegmentList::iterator neighborIterator;
  if (!access::isLeftHandedTraffic())
  {
    // sorting: right lanes are added at front, left lanes at back
    neighborIterator = roadSegment.drivableLaneSegments.end() - 1;
  }
  else
  {
    neighborIterator = roadSegment.drivableLaneSegments.begin();
  }
  laneInterval.end = neighborIterator->laneInterval.end;

  // let's ensure that the lane segment is neighbor of the provided road segment
  auto const neighborhood = getDirectNeighborhoodRelation(laneInterval.laneId, neighborIterator->laneInterval.laneId);
  if ((neighborhood != lane::ContactLocation::LEFT) && (neighborhood != lane::ContactLocation::RIGHT))
  {
    return physics::Distance(-1.);
  }

  // determine wrong way flag
  if (lane::isLaneDirectionPositive(neighborIterator->laneInterval.laneId)
      == lane::isLaneDirectionPositive(laneInterval.laneId))
  {
    laneInterval.wrongWay = neighborIterator->laneInterval.wrongWay;
  }
  else
  {
    laneInterval.wrongWay = !neighborIterator->laneInterval.wrongWay;
  }

  laneInterval = route::restrictIntervalFromBegin(laneInterval, distance);

  route::LaneSegment laneSegment;
  laneSegment.laneInterval = laneInterval;
  laneSegment.routeLaneOffset = neighborIterator->routeLaneOffset;

  if (!access::isLeftHandedTraffic())
  {
    laneSegment.rightNeighbor = neighborIterator->laneInterval.laneId;
    neighborIterator->leftNeighbor = laneInterval.laneId;
    updateRouteLaneOffset(false, laneSegment.routeLaneOffset, route);
    roadSegment.drivableLaneSegments.push_back(laneSegment);
  }
  else
  {
    laneSegment.leftNeighbor = neighborIterator->laneInterval.laneId;
    neighborIterator->rightNeighbor = laneInterval.laneId;
    updateRouteLaneOffset(true, laneSegment.routeLaneOffset, route);
    roadSegment.drivableLaneSegments.insert(roadSegment.drivableLaneSegments.begin(), laneSegment);
  }

  return route::calcLength(laneInterval);
}

bool addOpposingLaneToRoute(point::ParaPoint const &pointOnOppositeLane,
                            physics::Distance const &distanceOnWrongLane,
                            route::FullRoute &route,
                            physics::Distance &coveredDistance)
{
  coveredDistance = physics::Distance(0.);
  uint32_t segmentCounter = 0;
  point::ParaPoint startPoint = pointOnOppositeLane;

  if (route.roadSegments.empty())
  {
    return false;
  }

  auto startWayPoint = findWaypoint(startPoint, route);
  if (startWayPoint.isValid())
  {
    if (std::find_if(route.roadSegments[0].drivableLaneSegments.begin(),
                     route.roadSegments[0].drivableLaneSegments.end(),
                     [&startPoint](const route::LaneSegment &l) { return l.laneInterval.laneId == startPoint.laneId; })
        != route.roadSegments[0].drivableLaneSegments.end())
    {
      // point is already on route
      return false;
    }
  }

  while (coveredDistance < distanceOnWrongLane)
  {
    auto segmentDistance = route::addOpposingLaneSegmentToRoadSegment(
      startPoint, distanceOnWrongLane, route.roadSegments[segmentCounter], route);

    if (segmentDistance < physics::Distance(0.))
    {
      return false;
    }

    coveredDistance += segmentDistance;

    if (coveredDistance < distanceOnWrongLane)
    {
      lane::ContactLaneList successors;
      if (route::isRouteDirectionNegative(route.roadSegments[segmentCounter].drivableLaneSegments[0].laneInterval))
      {
        // get predecessor, if there are multiple its an intersection so stop
        successors = lane::getContactLanes(lane::getLane(startPoint.laneId), lane::ContactLocation::PREDECESSOR);
      }
      else
      {
        // get successor, if there are multiple its an intersection so stop
        successors = lane::getContactLanes(lane::getLane(startPoint.laneId), lane::ContactLocation::SUCCESSOR);
      }

      segmentCounter++;

      if ((successors.size() == 0) || (successors.size() > 1) || (segmentCounter == route.roadSegments.size()))
      {
        break;
      }

      startPoint.laneId = successors[0].toLane;
      startPoint.parametricOffset = route.roadSegments[segmentCounter].drivableLaneSegments[0].laneInterval.start;

      if (intersection::Intersection::isLanePartOfAnIntersection(startPoint.laneId))
      {
        break;
      }
    }
  }

  return true;
}

route::FullRoute getRouteExpandedToOppositeLanes(route::FullRoute const &route)
{
  return getRouteExpandedTo(route, RouteCreationMode::AllRoutableLanes);
}

route::FullRoute getRouteExpandedToAllNeighborLanes(route::FullRoute const &route)
{
  return getRouteExpandedTo(route, RouteCreationMode::AllNeighborLanes);
}

bool calculateBypassingRoute(route::FullRoute const &route, route::FullRoute &bypassingRoute)
{
  bypassingRoute = route::FullRoute();

  for (const auto &segment : route.roadSegments)
  {
    if (segment.drivableLaneSegments.empty())
    {
      // this should not happen
      // better avoid to passing
      return false;
    }

    // select the most outer relevant lane segment
    // that is for left-hand traffic the right most segment
    // that is for right-hand traffic the left most segment
    LaneSegment laneSegment;
    if (access::isLeftHandedTraffic())
    {
      laneSegment = segment.drivableLaneSegments.front();
    }
    else
    {
      laneSegment = segment.drivableLaneSegments.back();
    }

    // depending on the lane orientation and the driving direction (left-hand vs. right-hand)
    // the bypassingRoute has to use the left or right neighbor segments
    bool useLeftNeighbor = isRouteDirectionPositive(laneSegment.laneInterval);
    useLeftNeighbor = useLeftNeighbor ^ access::isLeftHandedTraffic();

    auto lane = lane::getLane(laneSegment.laneInterval.laneId);
    lane::ContactLaneList contactLanes;
    if (useLeftNeighbor)
    {
      contactLanes = lane::getContactLanes(lane, lane::ContactLocation::LEFT);
    }
    else
    {
      contactLanes = lane::getContactLanes(lane, lane::ContactLocation::RIGHT);
    }

    if (contactLanes.empty())
    {
      // there are no neighbors, thus no bypass option
      return false;
    }

    auto neighborLaneId = contactLanes.front().toLane;
    auto neighborLane = lane::getLane(neighborLaneId);
    if (lane::isLanePartOfAnIntersection(neighborLane))
    {
      return false;
    }

    route::LaneInterval neighborLaneInterval = laneSegment.laneInterval;
    neighborLaneInterval.laneId = neighborLaneId;
    if (lane.direction != neighborLane.direction)
    {
      neighborLaneInterval.wrongWay = !neighborLaneInterval.wrongWay;
    }
    route::appendLaneSegmentToRoute(neighborLaneInterval, bypassingRoute);
  }

  for (size_t i = 0; i < bypassingRoute.roadSegments.size(); ++i)
  {
    bypassingRoute.roadSegments[i].segmentCountFromDestination = bypassingRoute.roadSegments.size() - i;
  }

  return true;
}

// let's keep this for the moment like this with the internal inline function
// we will have to rework the border calculation in each case
// - add TRange instead of TParam...
// - consider the laneIntervals directly instead of the whole segment
inline LaneInterval
cutLaneIntervalAtEndByRoadSegmentParametricOffset(LaneInterval const &interval,
                                                  physics::ParametricValue const segmentParametricOffset)
{
  physics::ParametricValue laneParametricOffset = segmentParametricOffset;
  if (isRouteDirectionNegative(interval))
  {
    laneParametricOffset = physics::ParametricValue(1.) - segmentParametricOffset;
  }
  return cutIntervalAtEnd(interval, laneParametricOffset);
}

template <typename BorderType>
void getBorderOfRoadSegment(RoadSegment const &roadSegment,
                            BorderType &border,
                            physics::ParametricValue const parametricOffset)
{
  if (!roadSegment.drivableLaneSegments.empty())
  {
    getRightProjectedEdge(cutLaneIntervalAtEndByRoadSegmentParametricOffset(
                            roadSegment.drivableLaneSegments.front().laneInterval, parametricOffset),
                          border.right);
    getLeftProjectedEdge(cutLaneIntervalAtEndByRoadSegmentParametricOffset(
                           roadSegment.drivableLaneSegments.back().laneInterval, parametricOffset),
                         border.left);
  }
}

lane::ECEFBorder getECEFBorderOfRoadSegment(RoadSegment const &roadSegment,
                                            physics::ParametricValue const parametricOffset)
{
  lane::ECEFBorder result;
  getBorderOfRoadSegment(roadSegment, result, parametricOffset);
  return result;
}

lane::ENUBorder getENUBorderOfRoadSegment(RoadSegment const &roadSegment,
                                          physics::ParametricValue const parametricOffset)
{
  lane::ENUBorder result;
  getBorderOfRoadSegment(roadSegment, result, parametricOffset);
  return result;
}

lane::GeoBorder getGeoBorderOfRoadSegment(RoadSegment const &roadSegment,
                                          physics::ParametricValue const parametricOffset)
{
  lane::GeoBorder result;
  getBorderOfRoadSegment(roadSegment, result, parametricOffset);
  return result;
}

lane::ENUBorderList getENUBorderOfRoute(FullRoute const &route)
{
  lane::ENUBorderList enuBorderList;
  enuBorderList.reserve(route.roadSegments.size());
  for (auto const &roadSegment : route.roadSegments)
  {
    enuBorderList.push_back(getENUBorderOfRoadSegment(roadSegment));
  }
  return enuBorderList;
}

lane::ECEFBorderList getECEFBorderOfRoute(FullRoute const &route)
{
  lane::ECEFBorderList ecefBorderList;
  ecefBorderList.reserve(route.roadSegments.size());
  for (auto const &roadSegment : route.roadSegments)
  {
    ecefBorderList.push_back(getECEFBorderOfRoadSegment(roadSegment));
  }
  return ecefBorderList;
}

lane::GeoBorderList getGeoBorderOfRoute(FullRoute const &route)
{
  lane::GeoBorderList geoBorderList;
  geoBorderList.reserve(route.roadSegments.size());
  for (auto const &roadSegment : route.roadSegments)
  {
    geoBorderList.push_back(getGeoBorderOfRoadSegment(roadSegment));
  }
  return geoBorderList;
}

point::ENUHeading getENUHeadingOfRoute(match::Object const &object, FullRoute const &route)
{
  FindWaypointResult objectCenterWaypoint = findCenterWaypoint(object, route);
  if (!objectCenterWaypoint.isValid())
  {
    access::getLogger()->error("ad::map::route::getENUHeadingOfRoute: object not found in route {} {}", object, route);
    throw std::runtime_error("route::getENUHeadingOfRoute>> object not found in route");
  }
  // ensure the object center point is in
  auto const routeSectionAroundObject = getRouteSection(objectCenterWaypoint,
                                                        object.enuPosition.dimension.length,
                                                        object.enuPosition.dimension.length,
                                                        route,
                                                        RouteSectionCreationMode::AllRouteLanes);
  auto const routeENUBorders = getENUBorderOfRoute(routeSectionAroundObject);
  return lane::getENUHeading(routeENUBorders, object.enuPosition.centerPoint);
}

bool isObjectHeadingInRouteDirection(match::Object const &object, FullRoute const &route)
{
  auto const enuHeadingOfRoute = getENUHeadingOfRoute(object, route);
  // enforce normalization of angle difference
  const auto headingDifference
    = point::createENUHeading(static_cast<double>(std::fabs(enuHeadingOfRoute - object.enuPosition.heading)));
  if (static_cast<double>(std::fabs(headingDifference)) > M_PI / 2.)
  {
    return false;
  }
  return true;
}

} // namespace route
} // namespace map
} // namespace ad
