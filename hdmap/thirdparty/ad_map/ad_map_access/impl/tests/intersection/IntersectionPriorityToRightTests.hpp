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
 * @brief base class for PriorityToRight intersection tests
 */
struct IntersectionPriorityToRightTest : virtual SyntheticIntersectionTestBase
{
  virtual void prepareMap() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from west
 */
struct IntersectionPriorityToRightFromWestTest : IntersectionPriorityToRightTest
{
  virtual lane::LaneId getRouteStart() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from west to north
 */
struct IntersectionPriorityToRightWestToNorthTest : IntersectionPriorityToRightFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from west to east
 */
struct IntersectionPriorityToRightWestToEastTest : IntersectionPriorityToRightFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from west to south
 */
struct IntersectionPriorityToRightWestToSouthTest : IntersectionPriorityToRightFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from east
 */
struct IntersectionPriorityToRightFromEastTest : IntersectionPriorityToRightTest
{
  virtual lane::LaneId getRouteStart() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from east to north
 */
struct IntersectionPriorityToRightEastToNorthTest : IntersectionPriorityToRightFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from east to west
 */
struct IntersectionPriorityToRightEastToWestTest : IntersectionPriorityToRightFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from east to south
 */
struct IntersectionPriorityToRightEastToSouthTest : IntersectionPriorityToRightFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from north
 */
struct IntersectionPriorityToRightFromNorthTest : IntersectionPriorityToRightTest
{
  virtual lane::LaneId getRouteStart() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from north to east
 */
struct IntersectionPriorityToRightNorthToEastTest : IntersectionPriorityToRightFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from north to west
 */
struct IntersectionPriorityToRightNorthToWestTest : IntersectionPriorityToRightFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from north to south
 */
struct IntersectionPriorityToRightNorthToSouthTest : IntersectionPriorityToRightFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from south
 */
struct IntersectionPriorityToRightFromSouthTest : IntersectionPriorityToRightTest
{
  virtual lane::LaneId getRouteStart() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from south to east
 */
struct IntersectionPriorityToRightSouthToEastTest : IntersectionPriorityToRightFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from south to west
 */
struct IntersectionPriorityToRightSouthToWestTest : IntersectionPriorityToRightFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRight intersection tests coming from south to north
 */
struct IntersectionPriorityToRightSouthToNorthTest : IntersectionPriorityToRightFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

} // namespace map
} // namespace ad
