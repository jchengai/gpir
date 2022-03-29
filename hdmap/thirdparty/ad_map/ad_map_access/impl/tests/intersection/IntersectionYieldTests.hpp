// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#pragma once

#include "SyntheticIntersectionTestBase.hpp"

namespace ad {
namespace map {

/**
 * @brief base class for yield intersection tests
 */
struct IntersectionYieldTest : SyntheticIntersectionTestBase
{
  virtual void prepareMap() const override;
};

/**
 * @brief base class for yield intersection tests coming from west having to yield
 */
struct IntersectionYieldFromWestTest : IntersectionYieldTest
{
  virtual lane::LaneId getRouteStart() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for yield intersection tests coming from west to north having to yield
 */
struct IntersectionYieldWestToNorthTest : IntersectionYieldFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for yield intersection tests coming from west to east having to yield
 */
struct IntersectionYieldWestToEastTest : IntersectionYieldFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for yield intersection tests coming from west to south having to yield
 */
struct IntersectionYieldWestToSouthTest : IntersectionYieldFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for yield intersection tests coming from east having to yield
 */
struct IntersectionYieldFromEastTest : IntersectionYieldTest
{
  virtual lane::LaneId getRouteStart() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for yield intersection tests coming from east to north having to yield
 */
struct IntersectionYieldEastToNorthTest : IntersectionYieldFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for yield intersection tests coming from east to west having to yield
 */
struct IntersectionYieldEastToWestTest : IntersectionYieldFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for yield intersection tests coming from east to south having to yield
 */
struct IntersectionYieldEastToSouthTest : IntersectionYieldFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for yield intersection tests coming from north having right of way
 */
struct IntersectionYieldFromNorthTest : IntersectionYieldTest
{
  virtual lane::LaneId getRouteStart() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for yield intersection tests coming from north to east having right of way
 */
struct IntersectionYieldNorthToEastTest : IntersectionYieldFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for yield intersection tests coming from north to west having right of way
 */
struct IntersectionYieldNorthToWestTest : IntersectionYieldFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for yield intersection tests coming from north to south having right of way
 */
struct IntersectionYieldNorthToSouthTest : IntersectionYieldFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for yield intersection tests coming from south having right of way
 */
struct IntersectionYieldFromSouthTest : IntersectionYieldTest
{
  virtual lane::LaneId getRouteStart() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for yield intersection tests coming from south to east having right of way
 */
struct IntersectionYieldSouthToEastTest : IntersectionYieldFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for yield intersection tests coming from south to west having right of way
 */
struct IntersectionYieldSouthToWestTest : IntersectionYieldFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for yield intersection tests coming from south to north having right of way
 */
struct IntersectionYieldSouthToNorthTest : IntersectionYieldFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

} // namespace map
} // namespace ad
