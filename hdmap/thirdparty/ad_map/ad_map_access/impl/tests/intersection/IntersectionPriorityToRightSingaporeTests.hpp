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
 * @brief base class for PriorityToRightSingapore intersection tests
 */
struct IntersectionPriorityToRightSingaporeTest : SyntheticIntersectionTestBase
{
  virtual void prepareMap() const override;
  virtual intersection::IntersectionType expectedIntersectionType() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from west
 */
struct IntersectionPriorityToRightSingaporeFromWestTest : IntersectionPriorityToRightSingaporeTest
{
  virtual lane::LaneId getRouteStart() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from west to north
 */
struct IntersectionPriorityToRightSingaporeWestToNorthTest : IntersectionPriorityToRightSingaporeFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from west to east
 */
struct IntersectionPriorityToRightSingaporeWestToEastTest : IntersectionPriorityToRightSingaporeFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from west to south
 */
struct IntersectionPriorityToRightSingaporeWestToSouthTest : IntersectionPriorityToRightSingaporeFromWestTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from east
 */
struct IntersectionPriorityToRightSingaporeFromEastTest : IntersectionPriorityToRightSingaporeTest
{
  virtual lane::LaneId getRouteStart() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from east to north
 */
struct IntersectionPriorityToRightSingaporeEastToNorthTest : IntersectionPriorityToRightSingaporeFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from east to west
 */
struct IntersectionPriorityToRightSingaporeEastToWestTest : IntersectionPriorityToRightSingaporeFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from east to south
 */
struct IntersectionPriorityToRightSingaporeEastToSouthTest : IntersectionPriorityToRightSingaporeFromEastTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from north
 */
struct IntersectionPriorityToRightSingaporeFromNorthTest : IntersectionPriorityToRightSingaporeTest
{
  virtual lane::LaneId getRouteStart() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from north to east
 */
struct IntersectionPriorityToRightSingaporeNorthToEastTest : IntersectionPriorityToRightSingaporeFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from north to west
 */
struct IntersectionPriorityToRightSingaporeNorthToWestTest : IntersectionPriorityToRightSingaporeFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from north to south
 */
struct IntersectionPriorityToRightSingaporeNorthToSouthTest : IntersectionPriorityToRightSingaporeFromNorthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from south
 */
struct IntersectionPriorityToRightSingaporeFromSouthTest : IntersectionPriorityToRightSingaporeTest
{
  virtual lane::LaneId getRouteStart() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from south to east
 */
struct IntersectionPriorityToRightSingaporeSouthToEastTest : IntersectionPriorityToRightSingaporeFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from south to west
 */
struct IntersectionPriorityToRightSingaporeSouthToWestTest : IntersectionPriorityToRightSingaporeFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

/**
 * @brief base class for PriorityToRightSingapore intersection tests coming from south to north
 */
struct IntersectionPriorityToRightSingaporeSouthToNorthTest : IntersectionPriorityToRightSingaporeFromSouthTest
{
  virtual lane::LaneId getRouteEnd() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedIncomingLanesWithLowerPriority() const override;
  virtual lane::LaneIdSet expectedInternalLanesWithHigherPriority() const override;
  virtual lane::LaneIdSet expectedCrossingLanes() const override;
};

} // namespace map
} // namespace ad
