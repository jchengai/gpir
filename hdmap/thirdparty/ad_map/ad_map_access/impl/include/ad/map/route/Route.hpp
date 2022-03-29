// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <ad/physics/Distance.hpp>
#include <ad/physics/Duration.hpp>
#include <vector>

#include "ad/map/point/ParaPointList.hpp"
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

/**
 * @brief Implements routing on the lane network.
 */
class Route {
 public:
  /**
   * @brief Basic route description.
   */
  struct RawRoute {
    /** @brief the list of ParaPoints that contain the route planning output
     */
    point::ParaPointList paraPointList;
    /** @brief the distance covered by the route */
    physics::Distance routeDistance{0.};
    /** @brief the duration required by the route */
    physics::Duration routeDuration{0.};
  };

  /**
   * @brief Basic route description.
   */
  typedef std::vector<point::ParaPointList> BasicRoute;

  /**
   * @brief Routing type.
   */
  enum class Type {
    INVALID,                    ///< Invalid value.
    SHORTEST,                   ///< Shortest route by distance.
    SHORTEST_IGNORE_DIRECTION,  ///< Shortest route by distance, allow to drive
                                ///< also in lanes with other direction
  };

  /**
   * @brief Constructor. Calculates route between two points.
   * @param[in] start Start point.
   * @param[in] dest  Destination point.
   * @param[in] maxDistance maximum route search distance.
   * @param[in] maxDuration maximum route search duration.
   * @param[in] routingType   Type of the route to be calculated.
   */
  Route(const RoutingParaPoint &start, const RoutingParaPoint &dest,
        physics::Distance const &maxDistance,
        physics::Duration const &maxDuration, Type const &routingType);

  Route(Route const &) = delete;
  Route(Route const &&) = delete;
  Route &operator=(Route const &) = delete;
  Route &operator=(Route &&) = delete;

  /**
   * @brief Destructor.
   */
  virtual ~Route() = default;

  /** @returns Start point as ParaPoint. */
  const point::ParaPoint &getStart() const { return mStart.point; }

  /** @returns Start point as RoutingParaPoint. */
  const RoutingParaPoint &getRoutingStart() const { return mStart; }

  /** @returns Destination point as ParaPoint. */
  const point::ParaPoint &getDest() const { return mDest.point; }

  /** @returns Destination point as RoutingParaPoint. */
  const RoutingParaPoint &getRoutingDest() const { return mDest; }

  /** @returns Type of the route to be calculated. */
  Type getType() const { return mType; }

  /**
   * @returns true if route calculation ignores lane direction.
   */
  bool laneDirectionIsIgnored() const;

  /**
   * @brief Performs the routing.
   * @returns true if routing succeeded.
   */
  virtual bool calculate() = 0;

  /**
   * @returns true if calculated route is valid.
   */
  bool isValid() const { return mValid; }

  /** @returns Raw calculated route. */
  const RawRoute &getRawRoute(size_t const routeIndex = 0u) const;

  /** @returns Raw calculated route. */
  const std::vector<RawRoute> &getRawRoutes() const { return mRawRoutes; }

  /** @returns Calculated base route. */
  BasicRoute getBasicRoute(size_t const routeIndex = 0u) const;

  /** @returns All calculated basic routes. */
  std::vector<BasicRoute> getBasicRoutes() const;

 protected:
  //! Start point.
  RoutingParaPoint mStart;
  //! Destination point.
  RoutingParaPoint mDest;
  //! prediction distance to be used
  physics::Distance const mMaxDistance;
  //! prediction duration to be used
  physics::Duration const mMaxDuration;
  //! Type of the route to be calculated.
  Type mType;
  //! Indicates if calculation was successful.
  bool mValid;
  ///< Calculated routes.
  std::vector<RawRoute> mRawRoutes;
};

}  // namespace planning
}  // namespace route
}  // namespace map
}  // namespace ad
