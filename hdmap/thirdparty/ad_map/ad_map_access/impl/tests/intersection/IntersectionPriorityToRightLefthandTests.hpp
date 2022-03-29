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
 * @brief base class for PriorityToRightLefthand intersection tests
 */
struct IntersectionPriorityToRightLefthandTest : SyntheticIntersectionTestBase
{
  virtual void prepareMap() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from west
 */
struct IntersectionPriorityToRightLefthandFromWestTest : IntersectionPriorityToRightLefthandTest
{
  virtual lane::LaneId getRouteStart() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from west to north
 */
struct IntersectionPriorityToRightLefthandWestToNorthTest : IntersectionPriorityToRightLefthandFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from west to east
 */
struct IntersectionPriorityToRightLefthandWestToEastTest : IntersectionPriorityToRightLefthandFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from west to south
 */
struct IntersectionPriorityToRightLefthandWestToSouthTest : IntersectionPriorityToRightLefthandFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from east
 */
struct IntersectionPriorityToRightLefthandFromEastTest : IntersectionPriorityToRightLefthandTest
{
  virtual lane::LaneId getRouteStart() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from east to north
 */
struct IntersectionPriorityToRightLefthandEastToNorthTest : IntersectionPriorityToRightLefthandFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from east to west
 */
struct IntersectionPriorityToRightLefthandEastToWestTest : IntersectionPriorityToRightLefthandFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from east to south
 */
struct IntersectionPriorityToRightLefthandEastToSouthTest : IntersectionPriorityToRightLefthandFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from north
 */
struct IntersectionPriorityToRightLefthandFromNorthTest : IntersectionPriorityToRightLefthandTest
{
  virtual lane::LaneId getRouteStart() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from north to east
 */
struct IntersectionPriorityToRightLefthandNorthToEastTest : IntersectionPriorityToRightLefthandFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from north to west
 */
struct IntersectionPriorityToRightLefthandNorthToWestTest : IntersectionPriorityToRightLefthandFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from north to south
 */
struct IntersectionPriorityToRightLefthandNorthToSouthTest : IntersectionPriorityToRightLefthandFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from south
 */
struct IntersectionPriorityToRightLefthandFromSouthTest : IntersectionPriorityToRightLefthandTest
{
  virtual lane::LaneId getRouteStart() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from south to east
 */
struct IntersectionPriorityToRightLefthandSouthToEastTest : IntersectionPriorityToRightLefthandFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from south to west
 */
struct IntersectionPriorityToRightLefthandSouthToWestTest : IntersectionPriorityToRightLefthandFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightLefthand intersection tests coming from south to north
 */
struct IntersectionPriorityToRightLefthandSouthToNorthTest : IntersectionPriorityToRightLefthandFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

} // namespace map
} // namespace ad
