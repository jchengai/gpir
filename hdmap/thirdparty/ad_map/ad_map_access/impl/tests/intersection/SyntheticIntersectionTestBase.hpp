// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <ad/map/point/Operation.hpp>
#include <ad/map/test_support/ArtificialIntersectionTestBase.hpp>

namespace ad {
namespace map {

struct TrafficLightForTest
{
  landmark::LandmarkId id;
  landmark::TrafficLightType type{};
};

/**
 * @brief Common base for all intersection related tests that share the same geometry
 *
 * The maps only differ in the type of traffic regulation. Therefore, all locations
 * that have to be provided for the base class are the same.
 *
 * Derived classes only have to specify the map to be loaded (through overriding
 * prepareMap()
 */
struct SyntheticIntersectionTestBase : test_support::ArtificialIntersectionTestBase
{
  /**
   * @brief Override this to provide the type of intersection expected
   *
   * This depends also on the route through the intersection
   */
  virtual ad::map::intersection::IntersectionType expectedIntersectionType() const = 0;

  /**
   * @brief Override this to provide the incoming lanes with higher priority
   */
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const = 0;

  /**
   * @brief Override this to provide the incoming lanes with lower priority
   */
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const = 0;

  /**
   * @brief Override this to provide the internal lanes with higher priority
   */
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const = 0;

  /**
   * @brief Override this to provide the crossing lanes
   */
  virtual lane::LaneIdSet expectedCrossingLanes() const = 0;

  /**
   * @brief getter for the incoming lanes with higher priority
   *
   * Based on the expected* input and route calculations between the intersection arms.
   * This is compared by performBasicChecks() against the Intersection output.
   */
  lane::LaneIdSet getIncomingLanesWithHigherPriority() const;

  /**
   * @brief getter for the incoming para points with higher priority
   *
   * Based on the expected* input and route calculations between the intersection arms.
   * This is compared by performBasicChecks() against the Intersection output.
   */
  point::ParaPointList getIncomingParaPointsWithHigherPriority() const;

  /**
   * @brief getter for the incoming lanes with lower priority
   *
   * Based on the expected* input and route calculations between the intersection arms.
   * This is compared by performBasicChecks() against the Intersection output.
   */
  lane::LaneIdSet getIncomingLanesWithLowerPriority() const;

  /**
   * @brief getter for the incoming para points with lower priority
   *
   * Based on the expected* input and route calculations between the intersection arms.
   * This is compared by performBasicChecks() against the Intersection output.
   */
  point::ParaPointList getIncomingParaPointsWithLowerPriority() const;

  /**
   * @brief getter for the crossing lanes
   *
   * Based on the expected* input and route calculations between the intersection arms.
   * This is compared by performBasicChecks() against the Intersection output.
   */
  lane::LaneIdSet getCrossingLanes() const;

  /**
   * @brief support for comparison of two test vectors
   *
   * The vectors are passed by value because a copy is required in each case to be able to sort them
   * before the actual comparison
   */
  void compareVectors(point::ParaPointList left, point::ParaPointList right) const;

  /**
   * @brief support for comparison of two test sets
   *
   * The sets are passed by value because a copy is required in each case to be able to sort them
   * before the actual comparison
   */
  void compareLists(lane::LaneIdSet ids, lane::LaneIdSet otherIds) const;

  /**
   *  @brief perform the basic check for the intersection
   *
   *  You should at least write one test calling this basic checks to ensure the intersection rules are
   *  calculated as expected.
   */
  void performBasicChecks();

  /**
   *  @brief perform the basic check for the intersection with traffic lights.
   *
   *  this function tests if the number of traffic light and his IDs match for the expected route.
   *  this function calls performBasicChecks function to perform the basic intersection tests as well.
   *  @param trafficLights vector with the traffic light IDs for the route.
   */
  void performBasicTrafficLightsChecks(std::vector<TrafficLightForTest> TrafficLightForTests);

  TrafficLightForTest expectedTrafficLight(uint64_t landmarkId, landmark::TrafficLightType type) const;
  TrafficLightForTest expectedTrafficLight(uint64_t landmarkId) const;
};

} // namespace map
} // namespace ad
