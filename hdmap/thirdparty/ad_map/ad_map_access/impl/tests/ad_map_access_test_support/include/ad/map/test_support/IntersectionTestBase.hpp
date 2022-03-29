// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include <gtest/gtest.h>

#include "ad/map/intersection/Intersection.hpp"

/* @brief namespace ad */
namespace ad {
/* @brief namespace map */
namespace map {
/* @brief namespace test_support */
namespace test_support {

class IntersectionTestBase : public ::testing::Test
{
public:
  /**
   * @brief generic gtest SetUp() function
   *
   * Call this in your SetUp() function first.
   * This function:
   * - prepares the map
   * - plans a route (@see getRouteStart(), getRouteEnd())
   * - creates the intersection
   * - populates the RouteBasedValues below
   */
  virtual void SetUp() override;

  /**
   * @brief generic TearDown() function
   */
  virtual void TearDown() override;

  /**
   * @brief load map and set gnss reference
   */
  virtual void prepareMap() const = 0;

  /**
   * @brief getter for an unambiguous geo point of the intersection arm "from north"
   *
   * You have to overload this function and return the geo point in respect to your map coordinates.
   * If the intersection arm is not existing, just return point::GeoPoint();
   */
  virtual point::GeoPoint getGeoFromNorth() const = 0;

  /**
   * @brief getter for an unambiguous geo point of the intersection arm "to north"
   *
   * You have to overload this function and return the geo point in respect to your map coordinates
   * If the intersection arm is not existing, just return point::GeoPoint();
   */
  virtual point::GeoPoint getGeoToNorth() const = 0;

  /**
   * @brief getter for an unambiguous geo point of the intersection arm "from south"
   *
   * You have to overload this function and return the geo point in respect to your map coordinates
   * If the intersection arm is not existing, just return point::GeoPoint();
   */
  virtual point::GeoPoint getGeoFromSouth() const = 0;

  /**
   * @brief getter for an unambiguous geo point of the intersection arm "to south"
   *
   * You have to overload this function and return the geo point in respect to your map coordinates
   * If the intersection arm is not existing, just return point::GeoPoint();
   */
  virtual point::GeoPoint getGeoToSouth() const = 0;

  /**
   * @brief getter for an unambiguous geo point of the intersection arm "from west"
   *
   * You have to overload this function and return the geo point in respect to your map coordinates
   * If the intersection arm is not existing, just return point::GeoPoint();
   */
  virtual point::GeoPoint getGeoFromWest() const = 0;

  /**
   * @brief getter for an unambiguous geo point of the intersection arm "to west"
   *
   * You have to overload this function and return the geo point in respect to your map coordinates
   * If the intersection arm is not existing, just return point::GeoPoint();
   */
  virtual point::GeoPoint getGeoToWest() const = 0;

  /**
   * @brief getter for an unambiguous geo point of the intersection arm "from east"
   *
   * You have to overload this function and return the geo point in respect to your map coordinates
   * If the intersection arm is not existing, just return point::GeoPoint();
   */
  virtual point::GeoPoint getGeoFromEast() const = 0;

  /**
   * @brief getter for an unambiguous geo point of the intersection arm "to east"
   *
   * You have to overload this function and return the geo point in respect to your map coordinates
   * If the intersection arm is not existing, just return point::GeoPoint();
   */
  virtual point::GeoPoint getGeoToEast() const = 0;

  /**
   * @brief getter for the start of the route to be planned through the intersection
   *
   * You have to overload this function and return the lane id of the route start.
   * You just can select one of the logical member (mFrom*, e.g. mFromNorth)
   */
  virtual lane::LaneId getRouteStart() const = 0;

  /**
   * @brief getter for the end of the route to be planned through the intersection
   *
   * You have to overload this function and return the lane id of the route end.
   * You just can select one of the logical member (mTo*, e.g. mToSouth)
   */
  virtual lane::LaneId getRouteEnd() const = 0;

  lane::LaneIdSet createUnorderedLaneIdSet(lane::LaneId const &lane) const;
  lane::LaneIdSet createUnorderedLaneIdSet(std::vector<lane::LaneId> const &lanes) const;
  lane::LaneIdSet createUnorderedLaneIdSet(std::vector<lane::LaneIdSet> const &lanesVectors) const;

  point::ParaPointList createParaPointVector(std::vector<point::ParaPointList> const &paraPointsVector) const;

  /**
   * @brief getter for the internal lanes on the route between the intersection arms
   *
   * Based on the route calculation between every intersection entering arm getGeoFrom*()
   * to every intersection exiting arm getGeoTo*():
   * all lanes of these routes on road segments between first and last route segment
   */
  lane::LaneIdSet getInternalLanes() const;

  /**
   * @brief getter for the internal lanes on the route
   *
   * Based on the route calculation between getRouteStart() and getRouteEnd():
   * lanes of the first route segment
   */
  lane::LaneIdSet getIncomingLanesOnRoute() const
  {
    return mPlannedRouteValues.mIncomingLanes;
  }

  /**
   * @brief getter for the internal lanes on the route
   *
   * Based on the route calculation between getRouteStart() and getRouteEnd():
   * all lanes of route on road segments between first and last route segment
   */
  lane::LaneIdSet getInternalLanesOnRoute() const
  {
    return mPlannedRouteValues.mInternalLanes;
  }

  /**
   * @brief getter for the incoming para points on the route
   *
   * Based on the route calculation between getRouteStart() and getRouteEnd():
   * para points at the end of the first route segment
   */
  point::ParaPointList getIncomingParaPointsOnRoute() const
  {
    return mPlannedRouteValues.mIncomingParaPoints;
  }

  /**
   * @brief getter for the outgoing para points on the route
   *
   * Based on the route calculation between getRouteStart() and getRouteEnd():
   * para points at the begin of the last route segment
   */
  point::ParaPointList getOutgoingParaPointsOnRoute() const
  {
    return mPlannedRouteValues.mOutgoingParaPoints;
  }

  /**
   * @brief getter for the incoming lanes on the route between the intersection arms
   *
   * Based on the route calculation between every intersection entering arm getGeoFrom*()
   * to every intersection exiting arm getGeoTo*():
   * all lanes of these routes of every first road segment
   */
  lane::LaneIdSet getIncomingLanes() const;

  /**
   * @brief getter for the incoming lanes from north
   */
  lane::LaneIdSet getIncomingLanesNorth() const;
  /**
   * @brief getter for the incoming lanes from south
   */
  lane::LaneIdSet getIncomingLanesSouth() const;
  /**
   * @brief getter for the incoming lanes from west
   */
  lane::LaneIdSet getIncomingLanesWest() const;
  /**
   * @brief getter for the incoming lanes from east
   */
  lane::LaneIdSet getIncomingLanesEast() const;

  /**
   * @brief getter for the incoming para points on the route between the intersection arms
   *
   * Based on the route calculation between every intersection entering arm getGeoFrom*()
   * to every intersection exiting arm getGeoTo*():
   * all para points of these routes at the end of every first road segment
   */
  point::ParaPointList getIncomingParaPoints() const;

  /**
   * @brief the lane id of one of the lanes entering then intersection from north
   *
   * calculated from map matched geo coordinates @see getGeoFromNorth()
   */
  lane::LaneId mFromNorth;
  /**
   * @brief the lane id of one of the lanes entering then intersection to north
   *
   * calculated from map matched geo coordinates @see getGeoToNorth()
   */
  lane::LaneId mToNorth;
  /**
   * @brief the lane id of one of the lanes entering then intersection from south
   *
   * calculated from map matched geo coordinates @see getGeoFromSouth()
   */
  lane::LaneId mFromSouth;
  /**
   * @brief the lane id of one of the lanes entering then intersection to south
   *
   * calculated from map matched geo coordinates @see getGeoToSouth()
   */
  lane::LaneId mToSouth;
  /**
   * @brief the lane id of one of the lanes entering then intersection from west
   *
   * calculated from map matched geo coordinates @see getGeoFromWest()
   */
  lane::LaneId mFromWest;
  /**
   * @brief the lane id of one of the lanes entering then intersection to west
   *
   * calculated from map matched geo coordinates @see getGeoToWest()
   */
  lane::LaneId mToWest;
  /**
   * @brief the lane id of one of the lanes entering then intersection from east
   *
   * calculated from map matched geo coordinates @see getGeoFromEast()
   */
  lane::LaneId mFromEast;
  /**
   * @brief the lane id of one of the lanes entering then intersection to east
   *
   * calculated from map matched geo coordinates @see getGeoToEast()
   */
  lane::LaneId mToEast;

  /**
   * @brief struct to hold lane and para point information based on a planned route through the intersection
   */
  struct RouteBasedValues
  {
    /**
     * @brief the incoming para points
     *
     * all para points of the route at the end of the first road segment
     */
    point::ParaPointList mIncomingParaPoints;
    /**
     * @brief the incoming lanes
     *
     * all lanes of the route of the first road segment
     */
    lane::LaneIdSet mIncomingLanes;
    /**
     * @brief the internal lanes
     *
     * all lane intervals of route on road segments between first and last route segment
     */
    std::vector<route::LaneInterval> mInternalLaneIntervals;
    /**
     * @brief the internal lanes
     *
     * all lanes of route on road segments between first and last route segment
     */
    lane::LaneIdSet mInternalLanes;
    /**
     * @brief the outgoing lanes
     *
     * all lanes of the route of the last road segment
     */
    lane::LaneIdSet mOutgoingLanes;
    /**
     * @brief the outgoing para points
     *
     * all para points of the route at the start of the last road segment
     */
    point::ParaPointList mOutgoingParaPoints;
  };

  //! The route based values of the route from north to south
  RouteBasedValues mNorthToSouth;
  //! The route based values of the route from north to west
  RouteBasedValues mNorthToWest;
  //! The route based values of the route from north to east
  RouteBasedValues mNorthToEast;
  //! The route based values of the route from south to north
  RouteBasedValues mSouthToNorth;
  //! The route based values of the route from south to west
  RouteBasedValues mSouthToWest;
  //! The route based values of the route from south to east
  RouteBasedValues mSouthToEast;
  //! The route based values of the route from west to north
  RouteBasedValues mWestToNorth;
  //! The route based values of the route from west to south
  RouteBasedValues mWestToSouth;
  //! The route based values of the route from west to east
  RouteBasedValues mWestToEast;
  //! The route based values of the route from east to north
  RouteBasedValues mEastToNorth;
  //! The route based values of the route from east to south
  RouteBasedValues mEastToSouth;
  //! The route based values of the route from east to west
  RouteBasedValues mEastToWest;

  //! The ego route planned which was the basis for the creation of the intersection
  route::FullRoute mPlannedRoute;
  //! The route based values of the ego route
  RouteBasedValues mPlannedRouteValues;
  //! The intersection object created by the ego route
  intersection::IntersectionPtr mIntersection;

  /**
   * @brief provide a conversion from route parametric offset into a lane para point
   *
   * When placing objects in unit tests, it is desired to set the according to their logical
   * position within the route through the intersection (beginning of the intersection entry, end of the intersection
   * entry).
   */
  ::ad::map::point::ParaPoint
  getLaneParaPointFromRouteParametricOffset(::ad::map::lane::LaneId const laneId,
                                            ::ad::physics::ParametricValue const routeParametricOffset);

private:
  /**
   * @brief get the internal intersection lane segments for a given route.
   *
   * @param route start laneId.
   * @param route end laneId.
   * @return vector containing the internal intersection lanes.
   */
  RouteBasedValues getRouteBaseValues(lane::LaneId const &routeStartId, lane::LaneId const &routeEndId) const;

  /**
   * @brief plan a route.
   *
   * @param route start laneId.
   * @param route end laneId.
   * @return route.
   */
  route::FullRoute planRoute(lane::LaneId const &routeStartId, lane::LaneId const &routeEndId) const;

  /**
   * @brief filter impossible ways through the intersection.
   *
   * @param routeBasedValues RouteBasedValues.
   */
  void verifyImpossibleWays(RouteBasedValues &routeBasedValues);
};

} // namespace test_support
} // namespace map
} // namespace ad
