// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2019-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <utility>

#include "ad/map/lane/LaneOperation.hpp"
#include "ad/map/route/Route.hpp"
#include "ad/physics/Operation.hpp"

/* @brief namespace ad */
namespace ad {
/* @brief namespace map */
namespace map {
/* @brief namespace route */
namespace route {
/**
 * @namespace planning
 * @brief provides route planning capabilities on the road network of the map
 */
namespace planning {

static const physics::Distance cMinimumNeighborDistance{0.1};

static const physics::Duration cMinimumNeighborDuration{0.05};

static const physics::Speed cMinimumSpeed{10.};

/**
 * @brief Implements routing support functionality on the lane network.
 *
 * This class is used to expand a route by its reachable neighbors.
 * The routing cost data is defined by a template type to be defined by the actual routing class.
 */
template <class RoutingCostData> class RouteExpander : public Route
{
public:
  //! definition of the routing cost data
  struct RoutingCost
  {
    physics::Distance routeDistance{0.};
    physics::Duration routeDuration{0.};
    RoutingCostData costData{};
  };
  //! definition of the routing point
  typedef std::pair<RoutingParaPoint, RoutingCost> RoutingPoint;

  /**
   * @brief Constructor
   *
   * @param[in] start Start point.
   * @param[in] dest  Destination point.
   * @param[in] maxDistance maximum route search distance.
   * @param[in] maxDuration maximum route search duration.
   * @param[in] typ   Type of the route to be calculated.
   */
  RouteExpander(const RoutingParaPoint &start,
                const RoutingParaPoint &dest,
                physics::Distance const &maxDistance,
                physics::Duration const &maxDuration,
                Type const &routingType)
    : Route(start, dest, maxDistance, maxDuration, routingType)
  {
  }

  RouteExpander(RouteExpander const &) = delete;
  RouteExpander(RouteExpander const &&) = delete;
  RouteExpander &operator=(RouteExpander const &) = delete;
  RouteExpander &operator=(RouteExpander &&) = delete;

  /**
   * @brief destructor.
   */
  virtual ~RouteExpander() = default;

  /**
   * @brief Triggers the expansion of the given routing point \a origin to its valid neighborhood
   */
  void expandNeighbors(RoutingPoint const &origin);

protected:
  /**
   * @brief Definition of the reasons for route expansion
   */
  enum class ExpandReason
  {
    SameLaneNeighbor,     //!< expand the route by a neighbor point within the same lane
    LongitudinalNeighbor, //!< expand the route by a longitudinal neighbor lane point
    LateralNeighbor,      //!< expand the route by a lateral neighbor lane point
    Destination,          //!< expand the destination
  };

  /**
   * @brief a neighbor is added
   *
   * Override this function in the derived class to get notified on expanded neighbors.
   *
   * @param[in] originLane the lane object of the \a origin point
   * @param[in] origin the origin RoutingPoint provided to the ExpandNeighbors() function
   * @param[in] neighborLane the lane object of the \a neighbor point
   * @param[in] neighbor the neighbor RoutingPoint which is added (RoutingCostData not filled!)
   * @param[in] expandReason the reason why the origin was expanded to this neighbor
   */
  virtual void addNeighbor(lane::Lane::ConstPtr originLane,
                           RoutingPoint const &origin,
                           lane::Lane::ConstPtr neighborLane,
                           RoutingPoint const &neighbor,
                           ExpandReason const &expandReason)
    = 0;

  //! @returns \c true if the given origin point is the parametric start
  bool isStart(RoutingPoint const &origin)
  {
    return origin.first.point.parametricOffset == physics::ParametricValue(0.);
  }

  //! @returns \c true if the given origin point is the parametric end
  bool isEnd(RoutingPoint const &origin)
  {
    return origin.first.point.parametricOffset == physics::ParametricValue(1.);
  }

  //! @returns \c true if the given origin point on the given lane defines a positive movement
  bool isPositiveMovement(lane::Lane::ConstPtr lane, RoutingPoint const &origin)
  {
    return (laneDirectionIsIgnored() || isLaneDirectionPositive(*lane))
      && (origin.first.direction != RoutingDirection::NEGATIVE);
  }

  //! @returns \c true if the given origin point on the given lane defines a negative movement
  bool isNegativeMovement(lane::Lane::ConstPtr lane, RoutingPoint const &origin)
  {
    return (laneDirectionIsIgnored() || isLaneDirectionNegative(*lane))
      && (origin.first.direction != RoutingDirection::POSITIVE);
  }

  //! perform the expansion of the neighbor points on the same lane
  void expandSameLaneNeighbors(lane::Lane::ConstPtr lane, RoutingPoint const &origin);
  //! perform the expansion of the neighbor points in longitudinal (contacts: successor/predecessor) lane direction
  void expandLongitudinalNeighbors(lane::Lane::ConstPtr lane, RoutingPoint const &origin);
  //! perform the expansion of the neighbor points in lateral (contacts: left/right) lane direction
  void expandLateralNeighbors(lane::Lane::ConstPtr lane, RoutingPoint const &origin);

  //! create neighbor point and calculate the actual distance/duration to reach it
  RoutingPoint createNeighbor(lane::Lane::ConstPtr originLane,
                              RoutingPoint const &origin,
                              lane::Lane::ConstPtr neighborLane,
                              RoutingParaPoint neighborRoutingParaPoint);
  //! create longitudinal neighbor point with minimal actual distance/duration to reach
  RoutingPoint createLongitudinalNeighbor(RoutingPoint const &origin, RoutingParaPoint neighborRoutingParaPoint);
};

template <class RoutingCostData>
void RouteExpander<RoutingCostData>::expandNeighbors(
  typename RouteExpander<RoutingCostData>::RoutingPoint const &origin)
{
  lane::Lane::ConstPtr lane = lane::getLanePtr(origin.first.point.laneId);
  if (lane)
  {
    if ( // lane has to be routable to expand
      isRouteable(*lane)
      && ( // max distance and max duration are not yet reached
           ((origin.second.routeDistance < mMaxDistance) && (origin.second.routeDuration < mMaxDuration))
           // or we are within an intersection
           || lane::isLanePartOfAnIntersection(*lane)))
    {
      expandSameLaneNeighbors(lane, origin);
      expandLongitudinalNeighbors(lane, origin);
      expandLateralNeighbors(lane, origin);
    }
  }
  else
  {
    throw std::runtime_error("RouteExpander::ExpandNeighbors No lane!");
  }
}

template <class RoutingCostData>
typename RouteExpander<RoutingCostData>::RoutingPoint
RouteExpander<RoutingCostData>::createNeighbor(lane::Lane::ConstPtr originLane,
                                               RoutingPoint const &origin,
                                               lane::Lane::ConstPtr neighborLane,
                                               RoutingParaPoint neighborRoutingParaPoint)
{
  RoutingPoint neighbor;
  neighbor.first = neighborRoutingParaPoint;

  physics::Distance neighborDistance{0.};
  physics::Duration neighborDuration{0.};
  point::ECEFPoint pt_origin
    = getParametricPoint(*originLane, origin.first.point.parametricOffset, physics::ParametricValue(0.5));
  point::ECEFPoint pt_neighbor
    = getParametricPoint(*neighborLane, neighbor.first.point.parametricOffset, physics::ParametricValue(0.5));
  neighborDistance = point::distance(pt_neighbor, pt_origin);
  physics::ParametricRange drivingRange;
  if (origin.first.point.parametricOffset < neighbor.first.point.parametricOffset)
  {
    drivingRange.minimum = origin.first.point.parametricOffset;
    drivingRange.maximum = neighbor.first.point.parametricOffset;
  }
  else
  {
    drivingRange.minimum = neighbor.first.point.parametricOffset;
    drivingRange.maximum = origin.first.point.parametricOffset;
  }

  if (originLane == neighborLane)
  {
    neighborDuration = getDuration(*originLane, drivingRange);
  }
  else
  {
    auto const maxSpeed = std::max(getMaxSpeed(*originLane, drivingRange), cMinimumSpeed);
    neighborDuration = neighborDistance / maxSpeed;
  }
  neighborDistance = std::max(neighborDistance, cMinimumNeighborDistance);
  neighborDuration = std::max(neighborDuration, cMinimumNeighborDuration);
  neighbor.second.routeDistance = origin.second.routeDistance + neighborDistance;
  neighbor.second.routeDuration = origin.second.routeDuration + neighborDuration;
  return neighbor;
}

template <class RoutingCostData>
typename RouteExpander<RoutingCostData>::RoutingPoint
RouteExpander<RoutingCostData>::createLongitudinalNeighbor(RoutingPoint const &origin,
                                                           RoutingParaPoint neighborRoutingParaPoint)
{
  RoutingPoint neighbor;
  neighbor.first = neighborRoutingParaPoint;
  neighbor.second.routeDistance = origin.second.routeDistance + cMinimumNeighborDistance;
  neighbor.second.routeDuration = origin.second.routeDuration + cMinimumNeighborDuration;
  return neighbor;
}

template <class RoutingCostData>
void RouteExpander<RoutingCostData>::expandSameLaneNeighbors(
  lane::Lane::ConstPtr lane, typename RouteExpander<RoutingCostData>::RoutingPoint const &origin)
{
  if ((lane->id == getDest().laneId)
      && ((isPositiveMovement(lane, origin) && (origin.first.point.parametricOffset <= getDest().parametricOffset))
          || (isNegativeMovement(lane, origin) && (origin.first.point.parametricOffset >= getDest().parametricOffset))))
  {
    // approaching the destination from valid side
    auto const neighbor = createNeighbor(lane, origin, lane, getRoutingDest());
    addNeighbor(lane, origin, lane, neighbor, ExpandReason::Destination);
  }
  if (isPositiveMovement(lane, origin) && !isEnd(origin))
  {
    auto const neighbor = createNeighbor(
      lane, origin, lane, createRoutingParaPoint(lane->id, physics::ParametricValue(1.), origin.first.direction));
    addNeighbor(lane, origin, lane, neighbor, ExpandReason::SameLaneNeighbor);
  }
  if (isNegativeMovement(lane, origin) && !isStart(origin))
  {
    auto const neighbor = createNeighbor(
      lane, origin, lane, createRoutingParaPoint(lane->id, physics::ParametricValue(0.), origin.first.direction));
    addNeighbor(lane, origin, lane, neighbor, ExpandReason::SameLaneNeighbor);
  }
}

template <class RoutingCostData>
void RouteExpander<RoutingCostData>::expandLongitudinalNeighbors(
  lane::Lane::ConstPtr lane, typename RouteExpander<RoutingCostData>::RoutingPoint const &origin)
{
  lane::ContactLaneList contactLanes;
  if (isEnd(origin) && isPositiveMovement(lane, origin))
  {
    contactLanes = getContactLanes(*lane, lane::ContactLocation::SUCCESSOR);
  }
  else if (isStart(origin) && isNegativeMovement(lane, origin))
  {
    contactLanes = getContactLanes(*lane, lane::ContactLocation::PREDECESSOR);
  }
  for (auto contactLane : contactLanes)
  {
    lane::Lane::ConstPtr otherLane = lane::getLanePtr(contactLane.toLane);
    if (otherLane)
    {
      if (isRouteable(*otherLane))
      {
        lane::ContactLocation otherToLane = getContactLocation(*otherLane, lane->id);
        if (otherToLane == lane::ContactLocation::SUCCESSOR)
        {
          auto routingDirection = RoutingDirection::NEGATIVE;
          if (origin.first.direction == RoutingDirection::DONT_CARE)
          {
            routingDirection = RoutingDirection::DONT_CARE;
          }

          auto const neighbor = createLongitudinalNeighbor(
            origin, createRoutingParaPoint(otherLane->id, physics::ParametricValue(1.), routingDirection));

          addNeighbor(lane, origin, otherLane, neighbor, ExpandReason::LongitudinalNeighbor);
        }
        else if (otherToLane == lane::ContactLocation::PREDECESSOR)
        {
          auto routingDirection = RoutingDirection::POSITIVE;
          if (origin.first.direction == RoutingDirection::DONT_CARE)
          {
            routingDirection = RoutingDirection::DONT_CARE;
          }
          auto const neighbor = createLongitudinalNeighbor(
            origin, createRoutingParaPoint(otherLane->id, physics::ParametricValue(0.), routingDirection));

          addNeighbor(lane, origin, otherLane, neighbor, ExpandReason::LongitudinalNeighbor);
        }
        else
        {
          throw std::runtime_error("Other lane neither SUCCESSOR not PREDECESSOR!");
        }
      }
    }
    else
    {
      throw std::runtime_error("No other lane!");
    }
  }
}

template <class RoutingCostData>
void RouteExpander<RoutingCostData>::expandLateralNeighbors(
  lane::Lane::ConstPtr lane, typename RouteExpander<RoutingCostData>::RoutingPoint const &origin)
{
  for (auto const &contactLane : getContactLanes(*lane, {lane::ContactLocation::LEFT, lane::ContactLocation::RIGHT}))
  {
    lane::Lane::ConstPtr otherLane = lane::getLanePtr(contactLane.toLane);
    if (otherLane)
    {
      if (isRouteable(*otherLane))
      {
        if (laneDirectionIsIgnored() || (lane->direction == otherLane->direction))
        {
          auto const neighbor = createNeighbor(
            lane,
            origin,
            otherLane,
            createRoutingParaPoint(otherLane->id, origin.first.point.parametricOffset, origin.first.direction));
          addNeighbor(lane, origin, otherLane, neighbor, ExpandReason::LateralNeighbor);
        }
      }
    }
    else
    {
      throw std::runtime_error("No other lane!");
    }
  }
}

} // namespace planning
} // namespace route
} // namespace map
} // namespace ad
