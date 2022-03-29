// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "IntersectionYieldTests.hpp"
#include "MapSetup.hpp"

namespace ad {
namespace map {

void IntersectionYieldTest::prepareMap() const
{
  ::map_setup::prepareMapBasicYield();
}

lane::LaneId IntersectionYieldFromWestTest::getRouteStart() const
{
  return mFromWest;
}

intersection::IntersectionType IntersectionYieldFromWestTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::Yield;
}

lane::LaneId IntersectionYieldWestToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionYieldWestToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesSouth(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionYieldWestToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldWestToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToWest.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldWestToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToSouth.mInternalLanes});
}

TEST_F(IntersectionYieldWestToNorthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldWestToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionYieldWestToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionYieldWestToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return getIncomingLanesEast();
}

lane::LaneIdSet IntersectionYieldWestToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToWest.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldWestToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mEastToSouth.mInternalLanes});
}

TEST_F(IntersectionYieldWestToEastTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldWestToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionYieldWestToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionYieldWestToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionYieldWestToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToWest.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldWestToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mNorthToEast.mInternalLanes, mNorthToSouth.mInternalLanes, mEastToSouth.mInternalLanes});
}

TEST_F(IntersectionYieldWestToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldFromEastTest::getRouteStart() const
{
  return mFromEast;
}

intersection::IntersectionType IntersectionYieldFromEastTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::Yield;
}

lane::LaneId IntersectionYieldEastToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionYieldEastToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionYieldEastToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionYieldEastToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToWest.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldEastToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mWestToNorth.mInternalLanes, mSouthToNorth.mInternalLanes, mSouthToWest.mInternalLanes});
}

TEST_F(IntersectionYieldEastToNorthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldEastToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionYieldEastToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionYieldEastToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return getIncomingLanesWest();
}

lane::LaneIdSet IntersectionYieldEastToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToWest.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldEastToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mWestToNorth.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes});
}

TEST_F(IntersectionYieldEastToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldEastToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionYieldEastToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesSouth(), getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionYieldEastToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldEastToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToWest.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToNorth.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldEastToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToNorth.mInternalLanes});
}

TEST_F(IntersectionYieldEastToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldFromNorthTest::getRouteStart() const
{
  return mFromNorth;
}

intersection::IntersectionType IntersectionYieldFromNorthTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::HasWay;
}

lane::LaneId IntersectionYieldNorthToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionYieldNorthToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesSouth();
}

lane::LaneIdSet IntersectionYieldNorthToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionYieldNorthToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mSouthToEast.mInternalLanes, mSouthToNorth.mInternalLanes, mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldNorthToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mWestToNorth.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes});
}

TEST_F(IntersectionYieldNorthToEastTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldNorthToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionYieldNorthToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldNorthToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesEast(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionYieldNorthToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldNorthToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mSouthToWest.mInternalLanes, mEastToSouth.mInternalLanes, mEastToWest.mInternalLanes});
}

TEST_F(IntersectionYieldNorthToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldNorthToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionYieldNorthToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldNorthToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesEast(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionYieldNorthToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldNorthToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mSouthToWest.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes});
}

TEST_F(IntersectionYieldNorthToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldFromSouthTest::getRouteStart() const
{
  return mFromSouth;
}

intersection::IntersectionType IntersectionYieldFromSouthTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::HasWay;
}

lane::LaneId IntersectionYieldSouthToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionYieldSouthToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldSouthToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesEast(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionYieldSouthToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldSouthToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mNorthToEast.mInternalLanes, mWestToEast.mInternalLanes, mWestToNorth.mInternalLanes});
}

TEST_F(IntersectionYieldSouthToEastTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldSouthToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionYieldSouthToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesNorth();
}

lane::LaneIdSet IntersectionYieldSouthToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionYieldSouthToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mNorthToWest.mInternalLanes, mNorthToEast.mInternalLanes, mNorthToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldSouthToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes});
}

TEST_F(IntersectionYieldSouthToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldSouthToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionYieldSouthToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldSouthToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesEast(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionYieldSouthToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldSouthToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes});
}

TEST_F(IntersectionYieldSouthToNorthTest, basic_checks)
{
  performBasicChecks();
}

} // namespace map
} // namespace ad
