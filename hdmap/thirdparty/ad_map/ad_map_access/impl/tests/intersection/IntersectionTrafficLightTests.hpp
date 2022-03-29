// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "SyntheticIntersectionTestBase.hpp"

namespace ad {
namespace map {

/**
 * @brief base class for traffic light intersection tests
 *
 * Strictly speaking this is the base class for all traffic light related
 * tests located at Elfmorgenbruch-Ruschgraben ...
 */
struct IntersectionTrafficLightTest : virtual SyntheticIntersectionTestBase
{
  virtual void prepareMap() const override;

  virtual point::GeoPoint getGeoFromNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45787), point::Latitude(49.02084), point::Altitude(0)); // An der Tagweide
  }

  virtual point::GeoPoint getGeoToNorth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45782), point::Latitude(49.02073), point::Altitude(0)); // An der Tagweide
  }

  virtual point::GeoPoint getGeoFromSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45765), point::Latitude(49.02046), point::Altitude(0)); // Elfmorgenbruchstrasse
  }
  virtual point::GeoPoint getGeoToSouth() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45758), point::Latitude(49.02055), point::Altitude(0)); // Elfmorgenbruchstrasse
  }

  virtual point::GeoPoint getGeoFromWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45746), point::Latitude(49.02073), point::Altitude(0)); // Ruschgraben
  }

  virtual point::GeoPoint getGeoToWest() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45755), point::Latitude(49.02073), point::Altitude(0)); // Ruschgraben
  }

  virtual point::GeoPoint getGeoFromEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45807), point::Latitude(49.02059), point::Altitude(0)); // Am Storrenacker
  }

  virtual point::GeoPoint getGeoToEast() const override
  {
    return point::createGeoPoint(
      point::Longitude(8.45795), point::Latitude(49.02057), point::Altitude(0)); // Am Storrenacker
  }
};

/**
 * @brief base class for traffic light intersection tests coming from west
 */
struct IntersectionTrafficLightFromWestTest : virtual IntersectionTrafficLightTest
{
  virtual lane::LaneId getRouteStart() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from west to north
 */
struct IntersectionTrafficLightWestToNorthTest : IntersectionTrafficLightFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from west to east
 */
struct IntersectionTrafficLightWestToEastTest : IntersectionTrafficLightFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from west to south
 */
struct IntersectionTrafficLightWestToSouthTest : virtual IntersectionTrafficLightFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from east
 */
struct IntersectionTrafficLightFromEastTest : IntersectionTrafficLightTest
{
  virtual lane::LaneId getRouteStart() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from east to north
 */
struct IntersectionTrafficLightEastToNorthTest : IntersectionTrafficLightFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from east to west
 */
struct IntersectionTrafficLightEastToWestTest : IntersectionTrafficLightFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from east to south
 */
struct IntersectionTrafficLightEastToSouthTest : IntersectionTrafficLightFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from north
 */
struct IntersectionTrafficLightFromNorthTest : IntersectionTrafficLightTest
{
  virtual lane::LaneId getRouteStart() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from north to east
 */
struct IntersectionTrafficLightNorthToEastTest : IntersectionTrafficLightFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from north to west
 */
struct IntersectionTrafficLightNorthToWestTest : IntersectionTrafficLightFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from north to south
 */
struct IntersectionTrafficLightNorthToSouthTest : IntersectionTrafficLightFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from south
 */
struct IntersectionTrafficLightFromSouthTest : IntersectionTrafficLightTest
{
  virtual lane::LaneId getRouteStart() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from south to east
 */
struct IntersectionTrafficLightSouthToEastTest : IntersectionTrafficLightFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from south to west
 */
struct IntersectionTrafficLightSouthToWestTest : IntersectionTrafficLightFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for traffic light intersection tests coming from south to north
 */
struct IntersectionTrafficLightSouthToNorthTest : IntersectionTrafficLightFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

} // namespace map
} // namespace ad
