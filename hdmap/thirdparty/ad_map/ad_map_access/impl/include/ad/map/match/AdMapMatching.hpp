// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <list>

#include "ad/map/match/Types.hpp"
#include "ad/map/point/Operation.hpp"
#include "ad/map/route/Types.hpp"

/* @brief namespace admap */
namespace ad {
/* @brief namespace map */
namespace map {
/* @brief namespace match */
namespace match {

/**
 * @class AdMapMatching
 * @brief performs map matching of a given point to the map
 *
 * In general, there are multiple possible matches for a given point available,
 * especially if the point is e.g. located within an intersection area.
 * Therefore, map matching always returns a std::multimap of map matched
 * positions ordered by their probabilities.
 *
 * Besides the actual position, there are additional input parameters
 * influencing the probabilistic calculation. Observed heading or possible
 * routes of a vehicle/object can be provided to influence the map matching
 * result.
 */
class AdMapMatching {
 public:
  /**
   * @brief default constructor
   */
  AdMapMatching();

  /**
   * @brief destructor
   */
  ~AdMapMatching() = default;

  /**
   * @brief set the maximum probability multiplier for heading hints
   */
  void setMaxHeadingHintFactor(double newHeadingHintFactor) {
    mHeadingHintFactor = std::max(1., newHeadingHintFactor);
  }

  /**
   * @brief get the maximum probability multiplier for heading hints
   */
  double getMaxHeadingHintFactor() const { return mHeadingHintFactor; }

  /**
   * @brief set the probability multiplier for valid route hints
   */
  void setRouteHintFactor(double newRouteHintFactor) {
    mRouteHintFactor = std::max(1., newRouteHintFactor);
  }

  /**
   * @brief get the probability multiplier for valid route hints
   */
  double getRouteHintFactor() const { return mRouteHintFactor; }

  /**
   * @brief add a hint for the heading of the vehicle/object
   *
   * It is possible to provide hints on the heading of the vehicle/object to be
   * considered on map matching. This increases the map matching probabilities
   * for lanes with respective direction. The matches are multiplied with a
   * headingFactor:
   * 1. <= headingFactor <= getMaxHeadingHintFactor()
   *
   * @param[in] headingHint the heading hint of the object/vehicle to consider
   */
  void addHeadingHint(point::ECEFHeading const &headingHint) {
    mHeadingHints.push_back(headingHint);
  }

  /**
   * @brief add a hint for the heading of the vehicle/object
   *
   * It is possible to provide hints on the heading of the vehicle/object to be
   * considered on map matching. This increases the map matching probabilities
   * for lanes with respective direction.
   *
   * @param[in] yaw the yaw of the object/vehicle to consider
   * @param[in] enuReferencePoint the reference point of the corresponding
   * ENUCoordinate system
   */
  void addHeadingHint(point::ENUHeading const &yaw,
                      point::GeoPoint const &enuReferencePoint) {
    mHeadingHints.push_back(point::createECEFHeading(yaw, enuReferencePoint));
  }

  /**
   * @brief clears the list of heading hints
   */
  void clearHeadingHints() { mHeadingHints.clear(); }

  /**
   * @brief add a hint for the route of the vehicle/object
   *
   * This function allows to provide a hint for the current route of the object.
   * This increases the map matching probabilities for lanes along the route.
   *
   * @param[in] routeHint the route hint to consider
   */
  void addRouteHint(route::FullRoute const &routeHint) {
    mRouteHints.push_back(routeHint);
  }

  /**
   * @brief clears the list of route hints
   */
  void clearRouteHints() { mRouteHints.clear(); }

  /**
   * @brief clears route and heading hints at once
   */
  void clearHints() {
    clearRouteHints();
    clearHeadingHints();
  }

  /**
   * @brief get the map matched positions
   *
   * Calculate the map matched positions and return these.
   *
   * @param[in] geoPoint position to match against the map
   * @param[in] distance search radius around geoPoint to select a lane as a
   * match
   * @param[in] minProbabilty A probability threshold to be considered for the
   * results.
   */
  MapMatchedPositionConfidenceList getMapMatchedPositions(
      point::GeoPoint const &geoPoint, physics::Distance const &distance,
      physics::Probability const &minProbability) const;

  /**
   * @brief get the map matched positions
   *
   * Calculate the map matched positions and return these.
   *
   * @param[in] enuPoint position to match against the map in ENU coordinate
   * frame
   * @param[in] enuReferencePoint the enu reference point
   * @param[in] distance search radius around geoPoint to select a lane as a
   * match
   * @param[in] minProbabilty A probability threshold to be considered for the
   * results.
   */
  MapMatchedPositionConfidenceList getMapMatchedPositions(
      point::ENUPoint const &enuPoint, point::GeoPoint const &enuReferencePoint,
      physics::Distance const &distance,
      physics::Probability const &minProbability) const;

  /**
   * @brief get the map matched positions
   *
   * Calculate the map matched positions and return these.
   *
   * @param[in] enuPoint position to match against the map in ENU coordinate
   * frame
   * @param[in] distance search radius around geoPoint to select a lane as a
   * match
   * @param[in] minProbabilty A probability threshold to be considered for the
   * results.
   *
   * This function makes use of the globally set ENUReferencePoint
   * (see AdMapAccess::setENUReferencePoint())
   */
  MapMatchedPositionConfidenceList getMapMatchedPositions(
      point::ENUPoint const &enuPoint, physics::Distance const &distance,
      physics::Probability const &minProbability) const;

  /**
   * @brief get the map matched positions
   *
   * Calculate the map matched positions and return these.
   *
   * @param[in] enuObjectPosition object position, orientation, dimensions and
   * ENRReferencePoint to match against the map in ENU coordinate frame
   * @param[in] distance search radius around geoPoint to select a lane as a
   * match
   * @param[in] minProbabilty A probability threshold to be considered for the
   * results.
   *
   * The orientation of the ENUObjectPosition is set as heading hint before
   * matching and cleared afterwards This function makes use of the
   * ENUReferencePoint of the provided ENUObjectPosition. The dimensions of the
   * ENUObjectPosition is not required.
   */
  MapMatchedPositionConfidenceList getMapMatchedPositions(
      ENUObjectPosition const &enuObjectPosition,
      physics::Distance const &distance,
      physics::Probability const &minProbability);

  /**
   * @brief get the map matched bounding box
   *
   * Calculate the map matched bounding box.
   * This will calculate the map matched positions of all the corner points and
   * the center point In addition it will calculate all lane regions that are
   * covered by the bounding box by sampling the objects geometry in between
   * with the provided \c samplingDistance
   *
   * @param[in] enuObjectPosition object position, orientation, dimensions and
   * ENRReferencePoint to match against the map in ENU coordinate frame
   * @param[in] samplingDistance The step size to be used to perform map
   * matching in between the vehicle boundaries This parameter is heavily
   * influencing the performance of this function: A samplingDistance of 0.1 at
   * a car (3x5m) means 1500x map matching. With a distance of 1.0 we get only
   * 15x map matching.
   *
   * @returns the map matched bounding box of the object
   */
  MapMatchedObjectBoundingBox getMapMatchedBoundingBox(
      ENUObjectPosition const &enuObjectPosition,
      physics::Distance const &samplingDistance = physics::Distance(1.)) const;

  /**
   * @brief get the lane occupied regions from a list of ENUObjectPositionList
   *
   * Merge the lane occupied regions of the getMapMatchedBoundingBox() results
   * of all position entries. See getMapMatchedBoundingBox() for a detailed
   * description. For a correct handling of the inner borders crossed on
   * matching a bigger object, this function only works as expected if the the
   * provided enuObjectPositionList covers the whole object.
   *
   * @param[in] enuObjectPositionList list of ENUObjectPosition entries
   * @param[in] samplingDistance The step size to be used to perform map
   * matching in between the vehicle boundaries A samplingDistance of 0.1 at a
   * car (3x5m) means 1500x map matching. With a distance of 1.0 we get only 15x
   * map matching.
   *
   * @returns the map matched bounding box of the object
   */
  LaneOccupiedRegionList getLaneOccupiedRegions(
      ENUObjectPositionList enuObjectPositionList,
      physics::Distance const &samplingDistance = physics::Distance(1.)) const;

  /**
   * @brief Method to be called to retrieve the lane heading at a
   * mapMatchedPosition
   */
  point::ENUHeading getLaneENUHeading(
      MapMatchedPosition const &mapMatchedPosition) const;

  /**
   * @brief Spatial Lane Search.
   *        Returns all Lanes where any part of surface is less than specified
   * physics::Distance from given point.
   * @param[in] ecefPoint Point that is used as base for the search.
   * @param[in] distance Search radius.
   *
   * This static function doesn't make use of any matching hints.
   *
   * @returns Map matching results that satisfy search criteria.
   */
  static MapMatchedPositionConfidenceList findLanes(
      point::ECEFPoint const &ecefPoint, physics::Distance const &distance);

  /**
   * @brief Spatial Lane Search.
   *        Returns all Lanes where any part of surface is less than specified
   * physics::Distance from given point.
   * @param[in] geoPoint Point that is used as base for the search.
   * @param[in] distance Search radius.
   *
   * This static function doesn't make use of any matching hints.
   *
   * @returns Map matching results that satisfy search criteria. The individual
   * matching result probabilities are equal.
   */
  static MapMatchedPositionConfidenceList findLanes(
      point::GeoPoint const &geoPoint, physics::Distance const &distance);

  /**
   * @brief Spatial Lane Search.
   *        Returns the map matched position in respect to all Lanes of the
   * given route.
   * @param[in] ecefPoint Point that is used as base for the search.
   * @param[in] route The route providing the lane subset to be searched.
   *
   * This static function doesn't make use of any matching hints.
   *
   * @returns The individual matching result probabilities are relative to the
   * actual distance of the matchedPoint to the queryPoint.
   */
  static MapMatchedPositionConfidenceList findRouteLanes(
      point::ECEFPoint const &ecefPoint, route::FullRoute const &route);

 private:
  // Copy operators and constructors are deleted to avoid accidental copies
  AdMapMatching(AdMapMatching const &) = delete;
  AdMapMatching(AdMapMatching &&) = delete;
  AdMapMatching &operator=(AdMapMatching &&) = delete;
  AdMapMatching &operator=(AdMapMatching const &) = delete;

  static match::MapMatchedPositionConfidenceList findLanesInputChecked(
      point::ECEFPoint const &ecefPoint, physics::Distance const &distance);

  static match::MapMatchedPositionConfidenceList findLanesInputChecked(
      std::vector<lane::Lane::ConstPtr> const &relevantLanes,
      point::ECEFPoint const &ecefPoint, physics::Distance const &distance);

  static std::vector<lane::Lane::ConstPtr> getRelevantLanesInputChecked(
      point::ECEFPoint const &ecefPoint, physics::Distance const &distance);

  /**
   * @brief extract the mapMatchedPositions and write them into a map of
   * occuppied regions
   *
   * @param[in,out] laneOccupiedRegions vector containing the occupied regions
   * @param[in] mapMatchedPositions
   */

  void addLaneRegions(
      LaneOccupiedRegionList &laneOccupiedRegions,
      MapMatchedPositionConfidenceList const &mapMatchedPositions) const;
  void addLaneRegions(
      LaneOccupiedRegionList &laneOccupiedRegions,
      LaneOccupiedRegionList const &otherLaneOccupiedRegions) const;

  MapMatchedPositionConfidenceList considerMapMatchingHints(
      MapMatchedPositionConfidenceList const &mapMatchedPositions,
      physics::Probability const &minProbability) const;
  bool isLanePartOfRouteHints(lane::LaneId const &laneId) const;
  double getHeadingFactor(MapMatchedPosition const &matchedPosition) const;

  std::list<point::ECEFHeading> mHeadingHints;
  double mHeadingHintFactor{2.};
  std::list<route::FullRoute> mRouteHints;
  double mRouteHintFactor{10.};
};

}  // namespace match
}  // namespace map
}  // namespace ad
