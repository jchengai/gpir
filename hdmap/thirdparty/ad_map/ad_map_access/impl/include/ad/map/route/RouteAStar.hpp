// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <map>

#include "ad/map/route/RouteExpander.hpp"

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

/**
 * @brief cost data for route calculation
 */
struct RouteAstarScore {
  //! the estimated distance to destination
  physics::Distance estimatedDistanceToTarget{0.};
};

/**
 * @brief Implements routing on the lane network.
 */
class RouteAstar : public RouteExpander<RouteAstarScore> {
 public:
  using RouteExpander::RoutingCost;
  using RouteExpander::RoutingPoint;

  /**
   * @brief Constructor. Calculates route between two points.
   * @param[in] start Start point.
   * @param[in] dest  Destination point.
   * @param[in] maxDistance maximum route search distance.
   * @param[in] maxDuration maximum route search duration.
   * @param[in] typ   Type of the route to be calculated.
   */
  RouteAstar(const RoutingParaPoint &start, const RoutingParaPoint &dest,
             physics::Distance const &maxDistance,
             physics::Duration const &maxDuration, Type typ);

  /**
   * @brief Constructor. Calculates route between two points.
   * @param[in] start Start point.
   * @param[in] dest  Destination point.
   * @param[in] maxDistance maximum route search distance.
   * @param[in] typ   Type of the route to be calculated.
   */
  RouteAstar(const RoutingParaPoint &start, const RoutingParaPoint &dest,
             physics::Distance const &maxDistance, Type typ);

  /**
   * @brief Constructor. Calculates route between two points.
   * @param[in] start Start point.
   * @param[in] dest  Destination point.
   * @param[in] maxDuration maximum route search duration.
   * @param[in] typ   Type of the route to be calculated.
   */
  RouteAstar(const RoutingParaPoint &start, const RoutingParaPoint &dest,
             physics::Duration const &maxDuration, Type typ);

  /**
   * @brief Constructor. Calculates route between two points.
   * @param[in] start Start point.
   * @param[in] dest  Destination point.
   * @param[in] typ   Type of the route to be calculated.
   */
  RouteAstar(const RoutingParaPoint &start, const RoutingParaPoint &dest,
             Type typ);

  /**
   * @brief Calculates the route using A* algorithm.
   * @returns true if route is found.
   */
  bool calculate() override;

 private:
  /**
   * @brief Initialize the lane pointer members
   */
  void initLanePointer();

  /**
   * @brief Reimplemented from RouteExpander::AddNeighbor()
   */
  void addNeighbor(lane::Lane::ConstPtr originLane, RoutingPoint const &origin,
                   lane::Lane::ConstPtr neighborLane,
                   RoutingPoint const &neighbor,
                   ExpandReason const &expandReason) override;
  /**
   * @brief (under-)estimate the cost until the destination; required for A*
   * search criteria
   */
  physics::Distance costEstimate(lane::Lane::ConstPtr neighborLane,
                                 point::ParaPoint const &neighbor);

  /**
   * @brief reconstruct the path after search finished
   */
  void reconstructPath(RoutingPoint const &dest);

  /**
   * @brief the destination lane
   */
  lane::Lane::ConstPtr mDestLane;
  /**
   * @brief the start lane
   */
  lane::Lane::ConstPtr mStartLane;

  /**
   *  @brief typedef for a set of RoutingParaPoint
   */
  typedef std::set<RoutingParaPoint> RoutingParaPointSet;

  /**
   * @brief the already processed points (only process a point once)
   */
  RoutingParaPointSet mProcessedPoints;

  /**
   * @brief map holding the elements beeing processed
   */
  typedef std::map<RoutingParaPoint, RoutingCost> RoutingParaPointCostMap;
  RoutingParaPointCostMap mProcessingMap;

  /**
   * @brief map a point to its predecessor having least cost
   */
  typedef std::map<RoutingParaPoint, RoutingParaPoint> RoutingChainMap;
  RoutingChainMap mCameFrom;
};

}  // namespace planning
}  // namespace route
}  // namespace map
}  // namespace ad
