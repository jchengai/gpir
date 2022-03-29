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
struct IntersectionYieldLefthandTest : SyntheticIntersectionTestBase
{
  virtual void prepareMap() const override;
};

/**
 * @brief base class for yield intersection tests coming from west having to yield
 */
struct IntersectionYieldLefthandFromWestTest : IntersectionYieldLefthandTest
{
  virtual lane::LaneId getRouteStart() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for yield intersection tests coming from west to north having to yield
 */
struct IntersectionYieldLefthandWestToNorthTest : IntersectionYieldLefthandFromWestTest
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
struct IntersectionYieldLefthandWestToEastTest : IntersectionYieldLefthandFromWestTest
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
struct IntersectionYieldLefthandWestToSouthTest : IntersectionYieldLefthandFromWestTest
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
struct IntersectionYieldLefthandFromEastTest : IntersectionYieldLefthandTest
{
  virtual lane::LaneId getRouteStart() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for yield intersection tests coming from east to north having to yield
 */
struct IntersectionYieldLefthandEastToNorthTest : IntersectionYieldLefthandFromEastTest
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
struct IntersectionYieldLefthandEastToWestTest : IntersectionYieldLefthandFromEastTest
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
struct IntersectionYieldLefthandEastToSouthTest : IntersectionYieldLefthandFromEastTest
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
struct IntersectionYieldLefthandFromNorthTest : IntersectionYieldLefthandTest
{
  virtual lane::LaneId getRouteStart() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for yield intersection tests coming from north to east having right of way
 */
struct IntersectionYieldLefthandNorthToEastTest : IntersectionYieldLefthandFromNorthTest
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
struct IntersectionYieldLefthandNorthToWestTest : IntersectionYieldLefthandFromNorthTest
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
struct IntersectionYieldLefthandNorthToSouthTest : IntersectionYieldLefthandFromNorthTest
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
struct IntersectionYieldLefthandFromSouthTest : IntersectionYieldLefthandTest
{
  virtual lane::LaneId getRouteStart() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for yield intersection tests coming from south to east having right of way
 */
struct IntersectionYieldLefthandSouthToEastTest : IntersectionYieldLefthandFromSouthTest
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
struct IntersectionYieldLefthandSouthToWestTest : IntersectionYieldLefthandFromSouthTest
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
struct IntersectionYieldLefthandSouthToNorthTest : IntersectionYieldLefthandFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

} // namespace map
} // namespace ad
