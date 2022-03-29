// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "ad/map/config/PointOfInterest.hpp"
#include "ad/map/match/Object.hpp"
#include "ad/map/route/Route.hpp"
#include "ad/map/route/RouteOperation.hpp"
#include "ad/map/route/Routing.hpp"
#include "ad/map/route/Types.hpp"

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
 * @brief create a RoutingParaPoint
 *
 * @param[in] laneId  the lane id
 * @param[in] parametricOffset the parametric offset
 * @param[in] routingDirection the routing direction in respect to the lane
 * orientation Be aware: this might be different from the nominal driving
 * direction!
 */
RoutingParaPoint createRoutingPoint(
    lane::LaneId const &laneId,
    physics::ParametricValue const &parametricOffset,
    RoutingDirection const &routingDirection = RoutingDirection::DONT_CARE);

/**
 * @brief create a RoutingParaPoint
 *
 * @param[in] paraPoint the parametric point
 * @param[in] routingDirection the routing direction in respect to the lane
 * orientation Be aware: this might be different from the nominal driving
 * direction!
 */
RoutingParaPoint createRoutingPoint(
    point::ParaPoint const &paraPoint,
    RoutingDirection const &routingDirection = RoutingDirection::DONT_CARE);

/**
 * @brief create a RoutingParaPoint
 *
 * We select the routing point by either taking the maximum or minimum of the
 * occupied region Since occupied regions span over a certain area, the point is
 * selected in a way, that it is ensured that any other point within the region
 * can be reached by routing with the given routing direction
 *
 * @param[in] occupiedRegion the occupied region
 * @param[in] routingDirection the routing direction in respect to the lane
 * orientation Be aware: this might be different from the nominal driving
 * direction!
 */
RoutingParaPoint createRoutingPoint(
    match::LaneOccupiedRegion const &occupiedRegion,
    RoutingDirection const &routingDirection = RoutingDirection::DONT_CARE);

/**
 * @brief create a RoutingParaPoint
 *
 * @param[in] paraPoint the parametric point
 * @param[in] heading the heading to be respected
 */
RoutingParaPoint createRoutingPoint(point::ParaPoint const &paraPoint,
                                    point::ENUHeading const &heading);

/**
 * @brief create a RoutingParaPoint
 *
 * We select the routing point by either taking the maximum or minimum of the
 * occupied region Since occupied regions span over a certain area, the point is
 * selected in a way, that it is ensured that any other point within the region
 * can be reached by routing with the given routing direction
 *
 * @param[in] occupiedRegion the occupied region
 * @param[in] heading the heading to be respected
 */
RoutingParaPoint createRoutingPoint(
    match::LaneOccupiedRegion const &occupiedRegion,
    point::ENUHeading const &heading);

/** @brief Calculates route between two points.
 * @param[in] start Start point as RoutingParaPoint (Be aware: routing direction
 * in respect to lane orientation!).
 * @param[in] dest  Destination point as RoutingParaPoint (Be aware: routing
 * direction in respect to lane orientation!).
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 */
route::FullRoute planRoute(const RoutingParaPoint &start,
                           const RoutingParaPoint &dest,
                           RouteCreationMode const routeCreationMode =
                               RouteCreationMode::SameDrivingDirection);

/** @brief Calculates route between two points.
 *
 * Orientation at start/dest are not considered.
 *
 * @param[in] start Start point.
 * @param[in] dest  Destination point.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 */
inline route::FullRoute planRoute(const point::ParaPoint &start,
                                  const point::ParaPoint &dest,
                                  RouteCreationMode const routeCreationMode =
                                      RouteCreationMode::SameDrivingDirection) {
  return planRoute(createRoutingPoint(start), createRoutingPoint(dest),
                   routeCreationMode);
}

/** @brief Calculates route between two points.
 *
 * Orientation at dest is not considered.
 *
 * @param[in] start Start point.
 * @param[in] startHeading Heading at start point.
 * @param[in] dest  Destination point.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 */
inline route::FullRoute planRoute(const point::ParaPoint &start,
                                  point::ENUHeading const &startHeading,
                                  const point::ParaPoint &dest,
                                  RouteCreationMode const routeCreationMode =
                                      RouteCreationMode::SameDrivingDirection) {
  return planRoute(createRoutingPoint(start, startHeading),
                   createRoutingPoint(dest), routeCreationMode);
}

/** @brief Calculates route between two points.
 * @param[in] start Start point.
 * @param[in] startHeading Heading at start point.
 * @param[in] dest  Destination point.
 * @param[in] destHeading Heading at dest point.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 */
inline route::FullRoute planRoute(const point::ParaPoint &start,
                                  point::ENUHeading const &startHeading,
                                  const point::ParaPoint &dest,
                                  point::ENUHeading const &destHeading,
                                  RouteCreationMode const routeCreationMode =
                                      RouteCreationMode::SameDrivingDirection) {
  return planRoute(createRoutingPoint(start, startHeading),
                   createRoutingPoint(dest, destHeading), routeCreationMode);
}

/** @brief Calculates route between two points.
 * @param[in] start Start point as RoutingParaPoint (Be aware: routing direction
 * in respect to lane orientation!).
 * @param[in] dest  Destination point as geo point.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 */
route::FullRoute planRoute(const RoutingParaPoint &start,
                           const point::GeoPoint &dest,
                           RouteCreationMode const routeCreationMode =
                               RouteCreationMode::SameDrivingDirection);

/** @brief Calculates route between two points.
 * @param[in] start Start point as RoutingParaPoint (Be aware: routing direction
 * in respect to lane orientation!).
 * @param[in] dest  Destination point as ENU point.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 */
route::FullRoute planRoute(const RoutingParaPoint &start,
                           const point::ENUPoint &dest,
                           RouteCreationMode const routeCreationMode =
                               RouteCreationMode::SameDrivingDirection);

/** @brief Calculates route between two points.
 * @param[in] start Start point.
 * @param[in] dest  Destination point as geo point.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 */
inline route::FullRoute planRoute(const point::ParaPoint &start,
                                  const point::GeoPoint &dest,
                                  RouteCreationMode const routeCreationMode =
                                      RouteCreationMode::SameDrivingDirection) {
  return planRoute(createRoutingPoint(start), dest, routeCreationMode);
}

/** @brief Calculates route between two points.
 * @param[in] start Start point.
 * @param[in] startHeading Heading at start point.
 * @param[in] dest  Destination point as geo point.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 */
inline route::FullRoute planRoute(const point::ParaPoint &start,
                                  point::ENUHeading const &startHeading,
                                  const point::GeoPoint &dest,
                                  RouteCreationMode const routeCreationMode =
                                      RouteCreationMode::SameDrivingDirection) {
  return planRoute(createRoutingPoint(start, startHeading), dest,
                   routeCreationMode);
}

/** @brief Calculates route between two points.
 * @param[in] start Start point.
 * @param[in] dest  Destination point as point of interest.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 */
inline FullRoute planRoute(const point::ParaPoint &start,
                           const config::PointOfInterest &dest,
                           RouteCreationMode const routeCreationMode =
                               RouteCreationMode::SameDrivingDirection) {
  return planRoute(start, dest.geoPoint, routeCreationMode);
}

/** @brief Calculates route between two points considering supporting points on
 * the way.
 * @param[in] start Start point.
 * @param[in] dest Vector with supporting points as geo points to be visited on
 * the route. Last point in the list is the actual destination point.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 *
 * Be aware: Supporting points providing multiple map maptched positions (i.e.
 * the ones located within intersections) are discarded to ensure the proper
 * route is taken.
 */
FullRoute planRoute(const RoutingParaPoint &start,
                    const std::vector<point::GeoPoint> &dest,
                    RouteCreationMode const routeCreationMode =
                        RouteCreationMode::SameDrivingDirection);

/** @brief Calculates route between two points considering supporting points on
 * the way.
 * @param[in] start Start point.
 * @param[in] dest Vector with supporting points as ENU points to be visited on
 * the route. Last point in the list is the actual destination point.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 *
 * Be aware: Supporting points providing multiple map maptched positions (i.e.
 * the ones located within intersections) are discarded to ensure the proper
 * route is taken.
 */
FullRoute planRoute(const RoutingParaPoint &start,
                    const std::vector<point::ENUPoint> &dest,
                    RouteCreationMode const routeCreationMode =
                        RouteCreationMode::SameDrivingDirection);

/**
 * @brief Calculates route between two points considering supporting points on
 * the way
 * @param[in] start Start point.
 * @param[in] dest Vector with supporting points to be visited on the route.
 * Last point in the list is the actual destination point.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 */
FullRoute planRoute(const RoutingParaPoint &start,
                    std::vector<RoutingParaPoint> const &dest,
                    RouteCreationMode const routeCreationMode =
                        RouteCreationMode::SameDrivingDirection);

/** @brief Calculates route between two points considering supporting points on
 * the way.
 * @param[in] start Start point.
 * @param[in] startHeading Heading at start point.
 * @param[in] dest Vector with supporting points as geo points to be visited on
 * the route. Last point in the list is the actual destination point.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 */
inline route::FullRoute planRoute(const point::ParaPoint &start,
                                  point::ENUHeading const &startHeading,
                                  const std::vector<point::GeoPoint> &dest,
                                  RouteCreationMode const routeCreationMode =
                                      RouteCreationMode::SameDrivingDirection) {
  return planRoute(createRoutingPoint(start, startHeading), dest,
                   routeCreationMode);
}

/**
 * @brief perform route based prediction restricted by the prediction duration.
 * Note: Route predictions will not stop in the middle of an intersection.
 *   They continue until the intersection is left again.
 *
 * @param[in] start start point.
 * @param[in] predictionDuration duration when the prediction can be stopped.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 *
 * @return vector with all possible predicted routes.
 */
std::vector<route::FullRoute> predictRoutesOnDuration(
    const RoutingParaPoint &start, physics::Duration const &predictionDuration,
    RouteCreationMode const routeCreationMode =
        RouteCreationMode::SameDrivingDirection);

/**
 * @brief perform route based prediction restricted by the prediction distance.
 * Note: Route predictions will not stop in the middle of an intersection.
 *   They continue until the intersection is left again.
 *
 * @param[in] start start point.
 * @param[in] predictionDistance distance when the prediction can be stopped.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 *
 * @return vector with all possible predicted routes.
 */
std::vector<route::FullRoute> predictRoutesOnDistance(
    const RoutingParaPoint &start, physics::Distance const &predictionDistance,
    RouteCreationMode const routeCreationMode =
        RouteCreationMode::SameDrivingDirection);

/**
 * @brief perform route based prediction restricted by the prediction distance
 * and duration. Note: Route predictions will not stop in the middle of an
 * intersection. They continue until the intersection is left again.
 *
 * @param[in] start start point.
 * @param[in] predictionDistance distance when the prediction can be stopped.
 * @param[in] predictionDuration duration when the prediction can be stopped.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 *
 * @return vector with all possible predicted routes.
 */
std::vector<route::FullRoute> predictRoutes(
    const RoutingParaPoint &start, physics::Distance const &predictionDistance,
    physics::Duration const &predictionDuration,
    RouteCreationMode const routeCreationMode =
        RouteCreationMode::SameDrivingDirection);

/**
 * @brief perform route based prediction restricted by the prediction duration.
 * Note: Route predictions will not stop in the middle of an intersection.
 *   They continue until the intersection is left again.
 *
 * @param[in] start start point as map matched bounding box.
 * @param[in] predictionDuration duration when the prediction can be stopped.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 *
 * @return vector with all possible predicted routes.
 */
std::vector<route::FullRoute> predictRoutesOnDuration(
    const match::MapMatchedObjectBoundingBox &start,
    physics::Duration const &predictionDuration,
    RouteCreationMode const routeCreationMode =
        RouteCreationMode::SameDrivingDirection);

/**
 * @brief perform route based prediction restricted by the prediction distance.
 * Note: Route predictions will not stop in the middle of an intersection.
 *   They continue until the intersection is left again.
 *
 * @param[in] start start point as map matched bounding box.
 * @param[in] predictionDistance distance when the prediction can be stopped.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 *
 * @return vector with all possible predicted routes.
 */
std::vector<route::FullRoute> predictRoutesOnDistance(
    const match::MapMatchedObjectBoundingBox &start,
    physics::Distance const &predictionDistance,
    RouteCreationMode const routeCreationMode =
        RouteCreationMode::SameDrivingDirection);

/**
 * @brief perform route based prediction restricted by the prediction distance
 * and duration. Note: Route predictions will not stop in the middle of an
 * intersection. They continue until the intersection is left again.
 *
 * @param[in] start start point as map matched bounding box.
 * @param[in] predictionDistance distance when the prediction can be stopped.
 * @param[in] predictionDuration duration when the prediction can be stopped.
 * @param[in] routeCreationMode the mode of creating the route (default:
 * RouteCreationMode::SameDrivingDirection)
 *
 * @return vector with all possible predicted routes.
 */
std::vector<route::FullRoute> predictRoutes(
    const match::MapMatchedObjectBoundingBox &start,
    physics::Distance const &predictionDistance,
    physics::Duration const &predictionDuration,
    RouteCreationMode const routeCreationMode =
        RouteCreationMode::SameDrivingDirection);

/**
 * @brief Filter duplicated routes from a list of routes
 *
 * If one of the routes is a real sub-route of the other, the longer version of
 * the route is kept, the shorter dropped
 *
 * @param[in] fullRoutes list of full routes to be filtered
 *
 * @returns the filtered list of routes
 */
std::vector<FullRoute> filterDuplicatedRoutes(
    const std::vector<FullRoute> fullRoutes);

/**
 * @brief Calculate the connecting route between the the two objects
 *
 * For route calculations the route type
 * core::Route::Type::SHORTEST_IGNORE_DIRECTION is used. The prediction hints
 * are taken into account if no direct connecting route can be found in search
 * of a merge route. If no prediction hints are given, respective route
 * predictions are calculated internally.
 *
 * @param[in] startObject object at starting position
 * @param[in] destObject object at destination position
 * @param[in] maxDistance distance when the search can be stopped.
 * @param[in] maxDuration duration when the search can be stopped.
 * @param[in] startObjectPredictionHints route prediction hints for start object
 * (optional)
 * @param[in] destObjectPredictionHints route prediction hints for dest object
 * (optional)
 */
ConnectingRoute calculateConnectingRoute(
    const match::Object &startObject, const match::Object &destObject,
    physics::Distance const &maxDistance, physics::Duration const &maxDuration,
    std::vector<route::FullRoute> const &startObjectPredictionHints =
        std::vector<route::FullRoute>(),
    std::vector<route::FullRoute> const &destObjectPredictionHints =
        std::vector<route::FullRoute>());

/**
 * @brief Calculate the connecting route between the the two objects
 *
 * For route calculations the route type
 * core::Route::Type::SHORTEST_IGNORE_DIRECTION is used. The prediction hints
 * are taken into account if no direct connecting route can be found in search
 * of a merge route. If no prediction hints are given, respective route
 * predictions are calculated internally.
 *
 * @param[in] startObject object at starting position
 * @param[in] destObject object at destination position
 * @param[in] maxDistance distance when the search can be stopped.
 * @param[in] startObjectPredictionHints route prediction hints for start object
 * (optional)
 * @param[in] destObjectPredictionHints route prediction hints for dest object
 * (optional)
 */
ConnectingRoute calculateConnectingRoute(
    const match::Object &startObject, const match::Object &destObject,
    physics::Distance const &maxDistance,
    std::vector<route::FullRoute> const &startObjectPredictionHints =
        std::vector<route::FullRoute>(),
    std::vector<route::FullRoute> const &destObjectPredictionHints =
        std::vector<route::FullRoute>());

/**
 * @brief Calculate the connecting route between the the two objects
 *
 * For route calculations the route type
 * core::Route::Type::SHORTEST_IGNORE_DIRECTION is used. The prediction hints
 * are taken into account if no direct connecting route can be found in search
 * of a merge route. If no prediction hints are given, respective route
 * predictions are calculated internally.
 *
 * @param[in] startObject object at starting position
 * @param[in] destObject object at destination position
 * @param[in] maxDuration duration when the search can be stopped.
 * @param[in] startObjectPredictionHints route prediction hints for start object
 * (optional)
 * @param[in] destObjectPredictionHints route prediction hints for dest object
 * (optional)
 */
ConnectingRoute calculateConnectingRoute(
    const match::Object &startObject, const match::Object &destObject,
    physics::Duration const &maxDuration,
    std::vector<route::FullRoute> const &startObjectPredictionHints =
        std::vector<route::FullRoute>(),
    std::vector<route::FullRoute> const &destObjectPredictionHints =
        std::vector<route::FullRoute>());

/**
 * @brief update route planning counters of the route
 *
 * mainly used internally.
 */
void updateRoutePlanningCounters(route::FullRoute &route);

/**
 * @brief helper function to create a full route
 *
 * mainly used internally.
 */
FullRoute createFullRoute(const Route::RawRoute &rawRoute,
                          RouteCreationMode const routeCreationMode);

}  // namespace planning
}  // namespace route
}  // namespace map
}  // namespace ad
