// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/route/LaneIntervalOperation.hpp"

#include <algorithm>
#include "ad/map/access/Operation.hpp"
#include "ad/map/lane/LaneOperation.hpp"
#include "ad/map/point/Operation.hpp"

namespace ad {
namespace map {
namespace route {

point::ParaPoint getIntervalStart(FullRoute const &route, lane::LaneId const &laneId)
{
  point::ParaPoint result;
  for (auto const &roadSegment : route.roadSegments)
  {
    auto it = find_if(roadSegment.drivableLaneSegments.begin(),
                      roadSegment.drivableLaneSegments.end(),
                      [&laneId](const LaneSegment &l) { return l.laneInterval.laneId == laneId; });

    if (it != roadSegment.drivableLaneSegments.end())
    {
      result.laneId = laneId;
      result.parametricOffset = it->laneInterval.start;
      return result;
    }
  }

  throw std::invalid_argument("ad::map::route::getIntervalStart: laneId not found in route");
}

physics::ParametricValue
getSignedDistance(LaneInterval const &laneInterval, point::ParaPoint const &first, point::ParaPoint const &second)
{
  if ((first.laneId != second.laneId) || (first.laneId != laneInterval.laneId))
  {
    throw std::invalid_argument("ad::map::route::getSignedDistance: lane id's not matching");
  }

  if (isRouteDirectionPositive(laneInterval))
  {
    return second.parametricOffset - first.parametricOffset;
  }
  else
  {
    return first.parametricOffset - second.parametricOffset;
  }
}

physics::ParametricValue
getUnsignedDistance(LaneInterval const &laneInterval, point::ParaPoint const &first, point::ParaPoint const &second)
{
  if ((first.laneId != second.laneId) || (first.laneId != laneInterval.laneId))
  {
    throw std::invalid_argument("ad::map::route::getSignedDistance: lane id's not matching");
  }
  return std::fabs(first.parametricOffset - second.parametricOffset);
}

bool isRouteDirectionPositive(LaneInterval const &laneInterval)
{
  if (laneInterval.start == laneInterval.end)
  {
    return lane::isLaneDirectionPositive(laneInterval.laneId) ^ laneInterval.wrongWay;
  }
  else
  {
    return (laneInterval.start < laneInterval.end);
  }
}

bool isRouteDirectionAlignedWithDrivingDirection(LaneInterval const &laneInterval)
{
  if (isRouteDirectionPositive(laneInterval))
  {
    return lane::isLaneDirectionPositive(laneInterval.laneId);
  }
  else
  {
    return lane::isLaneDirectionNegative(laneInterval.laneId);
  }
}

physics::ParametricValue getProjectedParametricOffsetOnNeighborLane(LaneInterval const &currentInterval,
                                                                    LaneInterval const &neighborInterval,
                                                                    physics::ParametricValue const &parametricOffset)
{
  if (!lane::isSameOrDirectNeighbor(currentInterval.laneId, neighborInterval.laneId))
  {
    throw std::invalid_argument("ad::map::route::getProjectedParametricOffsetOnNeighborLane: lanes are not neighbors");
  }

  if (currentInterval.laneId == neighborInterval.laneId)
  {
    return parametricOffset;
  }

  // real neighbors
  auto currentLane = lane::getLane(currentInterval.laneId);
  auto neighborLane = lane::getLane(neighborInterval.laneId);
  auto leftNeighbors = lane::getContactLanes(currentLane, lane::ContactLocation::LEFT);
  auto rightNeighbors = lane::getContactLanes(currentLane, lane::ContactLocation::RIGHT);

  point::ECEFPoint leftECEFPoint;
  point::ECEFPoint rightECEFPoint;
  physics::ParametricValue offset;
  lane::projectParametricPointToEdges(currentLane, parametricOffset, leftECEFPoint, rightECEFPoint);

  if ((leftNeighbors.size() > 0) && (leftNeighbors[0].toLane == neighborInterval.laneId))
  {
    offset = ((point::findNearestPointOnEdge(neighborLane.edgeRight, leftECEFPoint)
               + point::findNearestPointOnEdge(neighborLane.edgeLeft, leftECEFPoint))
              / 2.);
  }
  else if ((rightNeighbors.size() > 0) && (rightNeighbors[0].toLane == neighborInterval.laneId))
  {
    offset = ((point::findNearestPointOnEdge(neighborLane.edgeRight, rightECEFPoint)
               + point::findNearestPointOnEdge(neighborLane.edgeLeft, rightECEFPoint))
              / 2.);
  }
  else
  {
    throw std::invalid_argument("ad::map::route::getProjectedParametricOffsetOnNeighborLane: lanes are not neighbors");
  }

  return offset;
}

physics::Distance calcLength(LaneInterval const &laneInterval)
{
  auto currentLane = lane::getLane(laneInterval.laneId);
  auto const resultDistance = currentLane.length * calcParametricLength(laneInterval);
  return resultDistance;
}

physics::Duration calcDuration(LaneInterval const &laneInterval)
{
  auto currentLane = lane::getLane(laneInterval.laneId);
  return lane::getDuration(currentLane, toParametricRange(laneInterval));
}

enum class EdgeType
{
  LEFT,
  RIGHT,
  LEFT_PROJECTED,
  RIGHT_PROJECTED
};

template <typename LaneEdge> void getEdge(LaneInterval const &laneInterval, EdgeType edgeType, LaneEdge &outputEdge)
{
  auto currentLane = lane::getLane(laneInterval.laneId);

  if (isRouteDirectionPositive(laneInterval))
  {
    if (edgeType == EdgeType::LEFT)
    {
      point::getParametricRange(currentLane.edgeLeft, toParametricRange(laneInterval), outputEdge, false);
    }
    else if (edgeType == EdgeType::LEFT_PROJECTED)
    {
      auto projectedInterval = laneInterval;
      projectedInterval.start = point::findNearestPointOnEdge(
        currentLane.edgeLeft,
        lane::getProjectedParametricPoint(currentLane, laneInterval.start, physics::ParametricValue(0.)));
      projectedInterval.end = point::findNearestPointOnEdge(
        currentLane.edgeLeft,
        lane::getProjectedParametricPoint(currentLane, laneInterval.end, physics::ParametricValue(0.)));
      point::getParametricRange(currentLane.edgeLeft, toParametricRange(projectedInterval), outputEdge, false);
    }
    else if (edgeType == EdgeType::RIGHT)
    {
      point::getParametricRange(currentLane.edgeRight, toParametricRange(laneInterval), outputEdge, false);
    }
    else if (edgeType == EdgeType::RIGHT_PROJECTED)
    {
      auto projectedInterval = laneInterval;
      projectedInterval.start = point::findNearestPointOnEdge(
        currentLane.edgeRight,
        lane::getProjectedParametricPoint(currentLane, laneInterval.start, physics::ParametricValue(1.)));
      projectedInterval.end = point::findNearestPointOnEdge(
        currentLane.edgeRight,
        lane::getProjectedParametricPoint(currentLane, laneInterval.end, physics::ParametricValue(1.)));
      point::getParametricRange(currentLane.edgeRight, toParametricRange(projectedInterval), outputEdge, false);
    }
  }
  else
  {
    if (edgeType == EdgeType::LEFT)
    {
      point::getParametricRange(currentLane.edgeRight, toParametricRange(laneInterval), outputEdge, true);
    }
    else if (edgeType == EdgeType::LEFT_PROJECTED)
    {
      auto projectedInterval = laneInterval;
      projectedInterval.start = point::findNearestPointOnEdge(
        currentLane.edgeRight,
        lane::getProjectedParametricPoint(currentLane, laneInterval.start, physics::ParametricValue(1.)));
      projectedInterval.end = point::findNearestPointOnEdge(
        currentLane.edgeRight,
        lane::getProjectedParametricPoint(currentLane, laneInterval.end, physics::ParametricValue(1.)));
      point::getParametricRange(currentLane.edgeRight, toParametricRange(projectedInterval), outputEdge, true);
    }
    else if (edgeType == EdgeType::RIGHT)
    {
      point::getParametricRange(currentLane.edgeLeft, toParametricRange(laneInterval), outputEdge, true);
    }
    else if (edgeType == EdgeType::RIGHT_PROJECTED)
    {
      auto projectedInterval = laneInterval;
      projectedInterval.start = point::findNearestPointOnEdge(
        currentLane.edgeLeft,
        lane::getProjectedParametricPoint(currentLane, laneInterval.start, physics::ParametricValue(0.)));
      projectedInterval.end = point::findNearestPointOnEdge(
        currentLane.edgeLeft,
        lane::getProjectedParametricPoint(currentLane, laneInterval.end, physics::ParametricValue(0.)));
      point::getParametricRange(currentLane.edgeLeft, toParametricRange(projectedInterval), outputEdge, true);
    }
  }
}

void getLeftEdge(LaneInterval const &laneInterval, point::ENUEdge &enuEdge)
{
  getEdge(laneInterval, EdgeType::LEFT, enuEdge);
}

void getRightEdge(LaneInterval const &laneInterval, point::ENUEdge &enuEdge)
{
  getEdge(laneInterval, EdgeType::RIGHT, enuEdge);
}

void getLeftProjectedEdge(LaneInterval const &laneInterval, point::ENUEdge &enuEdge)
{
  getEdge(laneInterval, EdgeType::LEFT_PROJECTED, enuEdge);
}

void getRightProjectedEdge(LaneInterval const &laneInterval, point::ENUEdge &enuEdge)
{
  getEdge(laneInterval, EdgeType::RIGHT_PROJECTED, enuEdge);
}

void getLeftEdge(LaneInterval const &laneInterval, point::GeoEdge &geoEdge)
{
  getEdge(laneInterval, EdgeType::LEFT, geoEdge);
}

void getRightEdge(LaneInterval const &laneInterval, point::GeoEdge &geoEdge)
{
  getEdge(laneInterval, EdgeType::RIGHT, geoEdge);
}

void getLeftProjectedEdge(LaneInterval const &laneInterval, point::GeoEdge &geoEdge)
{
  getEdge(laneInterval, EdgeType::LEFT_PROJECTED, geoEdge);
}

void getRightProjectedEdge(LaneInterval const &laneInterval, point::GeoEdge &geoEdge)
{
  getEdge(laneInterval, EdgeType::RIGHT_PROJECTED, geoEdge);
}

void getLeftEdge(LaneInterval const &laneInterval, point::ECEFEdge &ecefEdge)
{
  getEdge(laneInterval, EdgeType::LEFT, ecefEdge);
}

void getRightEdge(LaneInterval const &laneInterval, point::ECEFEdge &ecefEdge)
{
  getEdge(laneInterval, EdgeType::RIGHT, ecefEdge);
}

void getLeftProjectedEdge(LaneInterval const &laneInterval, point::ECEFEdge &ecefEdge)
{
  getEdge(laneInterval, EdgeType::LEFT_PROJECTED, ecefEdge);
}

void getRightProjectedEdge(LaneInterval const &laneInterval, point::ECEFEdge &ecefEdge)
{
  getEdge(laneInterval, EdgeType::RIGHT_PROJECTED, ecefEdge);
}

point::ENUEdge getRightENUEdge(LaneInterval const &laneInterval)
{
  point::ENUEdge enuEdge;
  getRightEdge(laneInterval, enuEdge);
  return enuEdge;
}

point::ECEFEdge getRightECEFEdge(LaneInterval const &laneInterval)
{
  point::ECEFEdge ecefEdge;
  getRightEdge(laneInterval, ecefEdge);
  return ecefEdge;
}

point::GeoEdge getRightGeoEdge(LaneInterval const &laneInterval)
{
  point::GeoEdge geoEdge;
  getRightEdge(laneInterval, geoEdge);
  return geoEdge;
}

point::ENUEdge getLeftENUEdge(LaneInterval const &laneInterval)
{
  point::ENUEdge enuEdge;
  getLeftEdge(laneInterval, enuEdge);
  return enuEdge;
}

point::ECEFEdge getLeftECEFEdge(LaneInterval const &laneInterval)
{
  point::ECEFEdge ecefEdge;
  getLeftEdge(laneInterval, ecefEdge);
  return ecefEdge;
}

point::GeoEdge getLeftGeoEdge(LaneInterval const &laneInterval)
{
  point::GeoEdge geoEdge;
  getLeftEdge(laneInterval, geoEdge);
  return geoEdge;
}

point::ENUEdge getRightProjectedENUEdge(LaneInterval const &laneInterval)
{
  point::ENUEdge enuEdge;
  getRightProjectedEdge(laneInterval, enuEdge);
  return enuEdge;
}

point::ECEFEdge getRightProjectedECEFEdge(LaneInterval const &laneInterval)
{
  point::ECEFEdge ecefEdge;
  getRightProjectedEdge(laneInterval, ecefEdge);
  return ecefEdge;
}

point::GeoEdge getRightProjectedGeoEdge(LaneInterval const &laneInterval)
{
  point::GeoEdge geoEdge;
  getRightProjectedEdge(laneInterval, geoEdge);
  return geoEdge;
}

point::ENUEdge getLeftProjectedENUEdge(LaneInterval const &laneInterval)
{
  point::ENUEdge enuEdge;
  getLeftProjectedEdge(laneInterval, enuEdge);
  return enuEdge;
}

point::ECEFEdge getLeftProjectedECEFEdge(LaneInterval const &laneInterval)
{
  point::ECEFEdge ecefEdge;
  getLeftProjectedEdge(laneInterval, ecefEdge);
  return ecefEdge;
}

point::GeoEdge getLeftProjectedGeoEdge(LaneInterval const &laneInterval)
{
  point::GeoEdge geoEdge;
  getLeftProjectedEdge(laneInterval, geoEdge);
  return geoEdge;
}

lane::GeoBorder getGeoBorder(LaneInterval const &laneInterval)
{
  lane::GeoBorder geoBorder;
  getLeftEdge(laneInterval, geoBorder.left);
  getRightEdge(laneInterval, geoBorder.right);
  return geoBorder;
}

lane::ECEFBorder getECEFBorder(LaneInterval const &laneInterval)
{
  lane::ECEFBorder ecefBorder;
  getLeftEdge(laneInterval, ecefBorder.left);
  getRightEdge(laneInterval, ecefBorder.right);
  return ecefBorder;
}

lane::ENUBorder getENUBorder(LaneInterval const &laneInterval)
{
  lane::ENUBorder enuBorder;
  getLeftEdge(laneInterval, enuBorder.left);
  getRightEdge(laneInterval, enuBorder.right);
  return enuBorder;
}

lane::ENUBorder getENUProjectedBorder(LaneInterval const &laneInterval)
{
  lane::ENUBorder enuBorder;
  getLeftProjectedEdge(laneInterval, enuBorder.left);
  getRightProjectedEdge(laneInterval, enuBorder.right);
  return enuBorder;
}

LaneInterval shortenIntervalFromBegin(LaneInterval const &laneInterval, physics::Distance const &distance)
{
  LaneInterval result = laneInterval;
  physics::ParametricValue delta(distance / lane::calcLength(laneInterval.laneId));
  if (isRouteDirectionPositive(laneInterval))
  {
    result.start = std::min(laneInterval.start + delta, laneInterval.end);
  }
  else
  {
    result.start = std::max(laneInterval.start - delta, laneInterval.end);
  }
  return result;
}

LaneInterval restrictIntervalFromBegin(LaneInterval const &laneInterval, physics::Distance const &distance)
{
  LaneInterval result = laneInterval;
  physics::ParametricValue delta(distance / lane::calcLength(laneInterval.laneId));
  if (isRouteDirectionNegative(laneInterval))
  {
    result.end = std::max(physics::ParametricValue(0.), laneInterval.start - delta);
  }
  else
  {
    result.end = std::min(physics::ParametricValue(1.), laneInterval.start + delta);
  }

  return result;
}

LaneInterval extendIntervalUntilEnd(LaneInterval const &laneInterval)
{
  LaneInterval resultInterval = laneInterval;
  if (isDegenerated(resultInterval))
  {
    // nothing to be done
  }
  else if (isRouteDirectionPositive(resultInterval))
  {
    resultInterval.end = physics::ParametricValue(1.0);
  }
  else
  {
    resultInterval.end = physics::ParametricValue(0.0);
  }
  return resultInterval;
}

LaneInterval shortenIntervalFromEnd(LaneInterval const &laneInterval, physics::Distance const &distance)
{
  LaneInterval result = laneInterval;
  physics::ParametricValue delta(distance / lane::calcLength(laneInterval.laneId));
  if (isRouteDirectionPositive(laneInterval))
  {
    result.end = std::max(laneInterval.end - delta, laneInterval.start);
  }
  else
  {
    result.end = std::min(laneInterval.end + delta, laneInterval.start);
  }
  return result;
}

LaneInterval extendIntervalFromStart(LaneInterval const &laneInterval, physics::Distance const &distance)
{
  if (isDegenerated(laneInterval))
  {
    return laneInterval;
  }

  LaneInterval resultInterval = laneInterval;
  physics::ParametricValue offset(distance / lane::calcLength(laneInterval.laneId));

  if (isRouteDirectionPositive(resultInterval))
  {
    resultInterval.start = std::max(physics::ParametricValue(0.0), laneInterval.start - offset);
  }
  else
  {
    resultInterval.start = std::min(physics::ParametricValue(1.0), laneInterval.start + offset);
  }
  return resultInterval;
}

LaneInterval extendIntervalFromEnd(LaneInterval const &laneInterval, physics::Distance const &distance)
{
  if (isDegenerated(laneInterval))
  {
    return laneInterval;
  }

  LaneInterval resultInterval = laneInterval;
  physics::ParametricValue offset(distance / lane::calcLength(laneInterval.laneId));

  if (isRouteDirectionPositive(resultInterval))
  {
    resultInterval.end = std::min(physics::ParametricValue(1.0), laneInterval.end + offset);
  }
  else
  {
    resultInterval.end = std::max(physics::ParametricValue(0.0), laneInterval.end - offset);
  }
  return resultInterval;
}

LaneInterval extendIntervalUntilStart(LaneInterval const &laneInterval)
{
  LaneInterval resultInterval = laneInterval;
  if (isDegenerated(resultInterval))
  {
    // nothing to be done
  }
  else if (isRouteDirectionPositive(resultInterval))
  {
    resultInterval.start = physics::ParametricValue(0.0);
  }
  else
  {
    resultInterval.start = physics::ParametricValue(1.0);
  }
  return resultInterval;
}

LaneInterval cutIntervalAtStart(LaneInterval const &laneInterval, physics::ParametricValue const &newIntervalStart)
{
  LaneInterval result = laneInterval;
  if (isWithinInterval(laneInterval, newIntervalStart))
  {
    result.start = newIntervalStart;
  }
  return result;
}

LaneInterval cutIntervalAtEnd(LaneInterval const &laneInterval, physics::ParametricValue const &newIntervalEnd)
{
  LaneInterval result = laneInterval;
  if (isWithinInterval(laneInterval, newIntervalEnd))
  {
    result.end = newIntervalEnd;
  }
  return result;
}

restriction::SpeedLimitList getSpeedLimits(LaneInterval const &laneInterval)
{
  auto lanePtr = lane::getLanePtr(laneInterval.laneId);
  return getSpeedLimits(*lanePtr, toParametricRange(laneInterval));
}

void getMetricRanges(LaneInterval const &laneInterval,
                     physics::MetricRange &lengthRange,
                     physics::MetricRange &widthRange)
{
  auto lanePtr = lane::getLanePtr(laneInterval.laneId);
  if (std::fabs(laneInterval.end - laneInterval.start) == physics::ParametricValue(1.0))
  {
    lengthRange = lanePtr->lengthRange;
    widthRange = lanePtr->widthRange;
  }
  else
  {
    auto const enuBorders = getENUProjectedBorder(laneInterval);
    auto const leftLength = calcLength(enuBorders.left);
    auto const rightLength = calcLength(enuBorders.right);
    lengthRange.minimum = std::min(leftLength, rightLength);
    lengthRange.maximum = std::max(leftLength, rightLength);

    if ((lanePtr->widthRange.maximum - lanePtr->widthRange.minimum) <= physics::Distance(.1))
    {
      widthRange = lanePtr->widthRange;
    }
    else
    {
      // take the effort on with range calculation only if there is significant difference within the lane
      auto const widthRangeResult = calculateWidthRange(enuBorders.left, leftLength, enuBorders.right, rightLength);
      widthRange = widthRangeResult.first;
    }
  }
}

} // namespace route
} // namespace map
} // namespace ad
