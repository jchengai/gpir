// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/route/RouteAStar.hpp"

#include <algorithm>
#include "ad/map/access/Logging.hpp"

#define DEBUG_OUTPUT 0

#if DEBUG_OUTPUT
#include <iostream>

namespace std {

inline ostream &operator<<(ostream &out, ::ad::map::route::planning::RouteAstar::RoutingPoint const &value)
{
  out << value.first.point << "|" << (int)value.first.direction << " ("
      << static_cast<double>(value.second.routeDistance) << ","
      << static_cast<double>(value.second.costData.estimatedDistanceToTarget) << ")";
  return out;
}

} // namespace std
#endif

namespace ad {
namespace map {
namespace route {
namespace planning {

RouteAstar::RouteAstar(const RoutingParaPoint &start,
                       const RoutingParaPoint &dest,
                       physics::Distance const &maxDistance,
                       physics::Duration const &maxDuration,
                       Type typ)
  : RouteExpander(start, dest, maxDistance, maxDuration, typ)
{
  initLanePointer();
}

RouteAstar::RouteAstar(const RoutingParaPoint &start,
                       const RoutingParaPoint &dest,
                       physics::Distance const &maxDistance,
                       Type typ)
  : RouteExpander(start, dest, maxDistance, physics::Duration::getMax(), typ)
{
  initLanePointer();
}

RouteAstar::RouteAstar(const RoutingParaPoint &start,
                       const RoutingParaPoint &dest,
                       physics::Duration const &maxDuration,
                       Type typ)
  : RouteExpander(start, dest, physics::Distance::getMax(), maxDuration, typ)
{
  initLanePointer();
}

RouteAstar::RouteAstar(const RoutingParaPoint &start, const RoutingParaPoint &dest, Type typ)
  : RouteExpander(start, dest, physics::Distance::getMax(), physics::Duration::getMax(), typ)
{
  initLanePointer();
}

void RouteAstar::initLanePointer()
{
  mDestLane = lane::getLanePtr(mDest.point.laneId);
  if (!mDestLane)
  {
    throw std::runtime_error("Dest lane not found!");
  }
  mStartLane = lane::getLanePtr(mStart.point.laneId);
  if (!mStartLane)
  {
    throw std::runtime_error("Start lane not found!");
  }
}

// https://en.wikipedia.org/wiki/A*_search_algorithm
bool RouteAstar::calculate()
{
  mProcessedPoints.clear();
  mProcessingMap.clear();
  mCameFrom.clear();
  mRawRoutes.clear();

  // A* working structures.
  // Initial values.
  RoutingCost cost;
  cost.costData.estimatedDistanceToTarget = costEstimate(mStartLane, mStart.point);
  mProcessingMap.insert({mStart, cost});
  // Run
  bool pathFound = false;
#if DEBUG_OUTPUT
  std::cout << "######### (typ:" << (int)mType << " #########"
            << " start: " << mStart.point << " dest: " << mDest.point << std::endl;
#endif
  while (!mProcessingMap.empty())
  {
    // sort the routing points by the smallest total cost
    struct FScoreCompare
    {
      bool operator()(RoutingPoint const &left, RoutingPoint const &right) const
      {
        return left.second.costData.estimatedDistanceToTarget < right.second.costData.estimatedDistanceToTarget;
      }
    };
    auto minimumCostIterator = std::min_element(mProcessingMap.begin(), mProcessingMap.end(), FScoreCompare());
    if (((mDest.direction == RoutingDirection::DONT_CARE) || (mDest.direction == minimumCostIterator->first.direction))
        && (minimumCostIterator->first.point == mDest.point))
    {
      reconstructPath(*minimumCostIterator);
      pathFound = true;
#if DEBUG_OUTPUT
      std::cout << "Target reached " << *minimumCostIterator << std::endl;
      for (auto const &point : getRawRoute().paraPointList)
      {
        std::cout << " " << point << std::endl;
      }
      std::cout << "#########" << std::endl;
#endif
      mProcessingMap.clear();
    }
    else
    {
      RoutingPoint const minimumValue = *minimumCostIterator;
      mProcessingMap.erase(minimumCostIterator);
      mProcessedPoints.insert(minimumValue.first);
#if DEBUG_OUTPUT
      std::cout << "Expanding: " << minimumValue << std::endl;
#endif
      expandNeighbors(minimumValue);
#if DEBUG_OUTPUT
      std::cout << "Results in " << std::endl;
      for (auto &element : mProcessingMap)
      {
        std::cout << " " << element << std::endl;
      }
      std::cout << "----------" << std::endl;
#endif
    }
  }

  mProcessedPoints.clear();
  mProcessingMap.clear();
  mCameFrom.clear();

  return pathFound;
}

physics::Distance RouteAstar::costEstimate(lane::Lane::ConstPtr neighborLane, point::ParaPoint const &neighbor)
{
  point::ECEFPoint pt_a = getParametricPoint(*neighborLane, neighbor.parametricOffset, physics::ParametricValue(0.5));
  point::ECEFPoint pt_b = getParametricPoint(*mDestLane, getDest().parametricOffset, physics::ParametricValue(0.5));
  physics::Distance d = distance(pt_a, pt_b);
  return d;
}

void RouteAstar::addNeighbor(lane::Lane::ConstPtr originLane,
                             RoutingPoint const &origin,
                             lane::Lane::ConstPtr neighborLane,
                             RoutingPoint const &neighbor,
                             ExpandReason const &expandReason)
{
  (void)originLane;
  (void)expandReason;

  if (mProcessedPoints.find(neighbor.first) == mProcessedPoints.end())
  {
    auto insertResult = mProcessingMap.insert(neighbor);
    if ( // insertion succeeded
      insertResult.second ||
      // actual distance of new neighbor is smaller than the found duplicate
      (neighbor.second.routeDistance < insertResult.first->second.routeDistance))
    {
      auto const estimatedDestCost = costEstimate(neighborLane, neighbor.first.point);
      insertResult.first->second.routeDistance = neighbor.second.routeDistance;
      insertResult.first->second.costData.estimatedDistanceToTarget = neighbor.second.routeDistance + estimatedDestCost;
      mCameFrom[neighbor.first] = origin.first;
#if DEBUG_OUTPUT
      std::cout << "Inserted: " << *insertResult.first << std::endl;
#endif
    }
  }
}

void RouteAstar::reconstructPath(RoutingPoint const &dest)
{
  RawRoute rawRoute;
  rawRoute.routeDistance = dest.second.routeDistance;
  rawRoute.routeDuration = dest.second.routeDuration;
  for (auto current = dest.first;;)
  {
    rawRoute.paraPointList.insert(rawRoute.paraPointList.begin(), current.point);
    auto it = mCameFrom.find(current);
    if (it == mCameFrom.end())
    {
      break;
    }
    else
    {
      current = it->second;
    }
  }
  mDest = dest.first;
  mValid = true;
  mRawRoutes.push_back(rawRoute);
}

} // namespace planning
} // namespace route
} // namespace map
} // namespace ad
