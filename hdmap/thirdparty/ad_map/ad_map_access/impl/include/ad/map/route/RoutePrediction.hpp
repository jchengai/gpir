// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <deque>
#include <map>

#include "ad/map/route/RouteExpander.hpp"
#include "ad/map/route/Routing.hpp"

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

struct RoutePredictionScore {};

/**
 * @brief Implements route prediction on the lane network.
 */
class RoutePrediction : public RouteExpander<RoutePredictionScore> {
 public:
  using RouteExpander::RoutingPoint;

  /**
   * @brief Constructor.
   *
   * Calculates route predictions from a starting point until either the given
   * predictionDistance or the given predictionDuration is reached. Note: Route
   * predictions will not stop in the middle of an intersection. They continue
   * until the intersection is left again.
   *
   * @param[in] start Start point.
   * @param[in] predictionDistance maximum prediction distance.
   * @param[in] predictionDuration maximum prediction duration.
   */
  RoutePrediction(const RoutingParaPoint &start,
                  physics::Distance const &predictionDistance,
                  physics::Duration const &predictionDuration);

  /**
   * @brief Constructor.
   *
   * Calculates route predictions from a starting point until the given
   * predictionDistance is reached. Note: Route predictions will not stop in the
   * middle of an intersection. They continue until the intersection is left
   * again.
   *
   * @param[in] start Start point.
   * @param[in] predictionDistance maximum prediction distance.
   */
  RoutePrediction(const RoutingParaPoint &start,
                  physics::Distance const &predictionDistance);

  /**
   * @brief Constructor.
   *
   * Calculates route predictions from a starting point until the given
   * predictionDuration is reached. Note: Route predictions will not stop in the
   * middle of an intersection. They continue until the intersection is left
   * again.
   *
   * @param[in] start Start point.
   * @param[in] predictionDuration maximum prediction duration.
   */
  RoutePrediction(const RoutingParaPoint &start,
                  physics::Duration const &predictionDuration);

  /**
   * @brief Calculates the route predictions using breadth search algorithm.
   */
  bool calculate() override;

 private:
  /**
   * @brief Reimplemented from RouteExpander::AddNeighbor()
   */
  void addNeighbor(lane::Lane::ConstPtr originLane, RoutingPoint const &origin,
                   lane::Lane::ConstPtr neighborLane,
                   RoutingPoint const &neighbor,
                   ExpandReason const &expandReason) override;
  /**
   * @brief Element of the routing tree
   */
  struct RouteTreeElement {
    //! Constructor.
    RouteTreeElement(RouteTreeElement const *parent,
                     RoutingPoint const &iRoutingPoint)
        : routingPoint(iRoutingPoint), theParent(parent) {}

    //! comparing route tree elements by their actual route-point to ensure
    //! children are unique
    struct RouteTreeElementCompare {
      bool operator()(const std::shared_ptr<RouteTreeElement> &left,
                      const std::shared_ptr<RouteTreeElement> &right) const {
        return left->routingPoint.first < right->routingPoint.first;
      }
    };

    typedef std::set<std::shared_ptr<RouteTreeElement>, RouteTreeElementCompare>
        RouteTreeElementUniqueRoutePointSet;

    //! the routing point represented by this tree element
    RoutingPoint routingPoint;
    //! the children of the tree element. If children are empty, the element is
    //! a leaf of the tree
    RouteTreeElementUniqueRoutePointSet children;
    //! pointer to the parent(s) of this, required for later reconstruction of
    //! the paths
    RouteTreeElement const *theParent;
  };

  /**
   * @brief reconstruct the path after search finished
   */
  void reconstructPaths();

  //! the tree root element
  std::shared_ptr<RouteTreeElement> mRouteTreeRoot;

  /**
   * @brief the already processed transitions from -> {to} (only process a
   * transition once)
   */
  std::map<RoutingParaPoint,
           RouteTreeElement::RouteTreeElementUniqueRoutePointSet>
      mProcessedTransitions;

  //! the list of elements to be processed
  std::deque<std::shared_ptr<RouteTreeElement>> mElementsToProcess;
};

}  // namespace planning
}  // namespace route
}  // namespace map
}  // namespace ad
