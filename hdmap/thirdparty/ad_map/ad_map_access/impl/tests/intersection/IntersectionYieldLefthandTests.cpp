// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "IntersectionYieldLefthandTests.hpp"
#include "MapSetup.hpp"

namespace ad {
namespace map {

void IntersectionYieldLefthandTest::prepareMap() const
{
  ::map_setup::prepareMapBasicYieldLefthand();
}

lane::LaneId IntersectionYieldLefthandFromWestTest::getRouteStart() const
{
  return mFromWest;
}

intersection::IntersectionType IntersectionYieldLefthandFromWestTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::Yield;
}

lane::LaneId IntersectionYieldLefthandWestToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionYieldLefthandWestToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionYieldLefthandWestToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionYieldLefthandWestToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldLefthandWestToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mEastToNorth.mInternalLanes, mSouthToEast.mInternalLanes, mSouthToNorth.mInternalLanes});
}

TEST_F(IntersectionYieldLefthandWestToNorthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldLefthandWestToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionYieldLefthandWestToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionYieldLefthandWestToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return getIncomingLanesEast();
}

lane::LaneIdSet IntersectionYieldLefthandWestToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldLefthandWestToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mEastToNorth.mInternalLanes});
}

TEST_F(IntersectionYieldLefthandWestToEastTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldLefthandWestToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionYieldLefthandWestToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesSouth(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionYieldLefthandWestToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldLefthandWestToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldLefthandWestToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes});
}

TEST_F(IntersectionYieldLefthandWestToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldLefthandFromEastTest::getRouteStart() const
{
  return mFromEast;
}

intersection::IntersectionType IntersectionYieldLefthandFromEastTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::Yield;
}

lane::LaneId IntersectionYieldLefthandEastToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionYieldLefthandEastToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesSouth(), getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionYieldLefthandEastToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldLefthandEastToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToNorth.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldLefthandEastToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mNorthToWest.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToNorth.mInternalLanes});
}

TEST_F(IntersectionYieldLefthandEastToNorthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldLefthandEastToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionYieldLefthandEastToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionYieldLefthandEastToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return getIncomingLanesWest();
}

lane::LaneIdSet IntersectionYieldLefthandEastToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldLefthandEastToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mWestToSouth.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes});
}

TEST_F(IntersectionYieldLefthandEastToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldLefthandEastToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionYieldLefthandEastToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionYieldLefthandEastToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionYieldLefthandEastToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldLefthandEastToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mWestToSouth.mInternalLanes, mNorthToWest.mInternalLanes, mNorthToSouth.mInternalLanes});
}

TEST_F(IntersectionYieldLefthandEastToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldLefthandFromNorthTest::getRouteStart() const
{
  return mFromNorth;
}

intersection::IntersectionType IntersectionYieldLefthandFromNorthTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::HasWay;
}

lane::LaneId IntersectionYieldLefthandNorthToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionYieldLefthandNorthToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldLefthandNorthToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesEast(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionYieldLefthandNorthToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldLefthandNorthToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mWestToSouth.mInternalLanes, mWestToEast.mInternalLanes, mSouthToEast.mInternalLanes});
}

TEST_F(IntersectionYieldLefthandNorthToEastTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldLefthandNorthToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionYieldLefthandNorthToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesSouth();
}

lane::LaneIdSet IntersectionYieldLefthandNorthToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionYieldLefthandNorthToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mSouthToWest.mInternalLanes, mSouthToEast.mInternalLanes, mSouthToNorth.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldLefthandNorthToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mWestToSouth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes});
}

TEST_F(IntersectionYieldLefthandNorthToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldLefthandNorthToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionYieldLefthandNorthToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldLefthandNorthToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesEast(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionYieldLefthandNorthToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldLefthandNorthToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mSouthToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes});
}

TEST_F(IntersectionYieldLefthandNorthToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldLefthandFromSouthTest::getRouteStart() const
{
  return mFromSouth;
}

intersection::IntersectionType IntersectionYieldLefthandFromSouthTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::HasWay;
}

lane::LaneId IntersectionYieldLefthandSouthToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionYieldLefthandSouthToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesNorth();
}

lane::LaneIdSet IntersectionYieldLefthandSouthToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionYieldLefthandSouthToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mNorthToWest.mInternalLanes, mNorthToEast.mInternalLanes, mNorthToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionYieldLefthandSouthToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mWestToNorth.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes});
}

TEST_F(IntersectionYieldLefthandSouthToEastTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldLefthandSouthToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionYieldLefthandSouthToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldLefthandSouthToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesEast(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionYieldLefthandSouthToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldLefthandSouthToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mNorthToWest.mInternalLanes, mEastToWest.mInternalLanes, mEastToNorth.mInternalLanes});
}

TEST_F(IntersectionYieldLefthandSouthToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionYieldLefthandSouthToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionYieldLefthandSouthToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldLefthandSouthToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesEast(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionYieldLefthandSouthToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionYieldLefthandSouthToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mNorthToWest.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes});
}

TEST_F(IntersectionYieldLefthandSouthToNorthTest, basic_checks)
{
  performBasicChecks();
}

} // namespace map
} // namespace ad
