// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "IntersectionPriorityToRightLefthandTests.hpp"
#include "MapSetup.hpp"

namespace ad {
namespace map {

void IntersectionPriorityToRightLefthandTest::prepareMap() const
{
  ::map_setup::prepareMapBasicPriorityToRightLefthand();
}

intersection::IntersectionType IntersectionPriorityToRightLefthandTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::PriorityToRight;
}

lane::LaneId IntersectionPriorityToRightLefthandFromWestTest::getRouteStart() const
{
  return mFromWest;
}

lane::LaneId IntersectionPriorityToRightLefthandWestToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionPriorityToRightLefthandWestToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesSouth();
}

lane::LaneIdSet IntersectionPriorityToRightLefthandWestToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandWestToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mSouthToEast.mInternalLanes, mSouthToNorth.mInternalLanes, mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandWestToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mEastToNorth.mInternalLanes, mSouthToEast.mInternalLanes, mSouthToNorth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightLefthandWestToNorthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightLefthandWestToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionPriorityToRightLefthandWestToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesSouth();
}

lane::LaneIdSet IntersectionPriorityToRightLefthandWestToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesEast(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandWestToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mSouthToNorth.mInternalLanes, mSouthToEast.mInternalLanes, mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandWestToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToEast.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightLefthandWestToEastTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightLefthandWestToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionPriorityToRightLefthandWestToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesEast(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandWestToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return getIncomingLanesNorth();
}

lane::LaneIdSet IntersectionPriorityToRightLefthandWestToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mSouthToNorth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandWestToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mNorthToSouth.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightLefthandWestToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightLefthandFromNorthTest::getRouteStart() const
{
  return mFromNorth;
}

lane::LaneId IntersectionPriorityToRightLefthandNorthToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionPriorityToRightLefthandNorthToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesWest();
}

lane::LaneIdSet IntersectionPriorityToRightLefthandNorthToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesEast(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandNorthToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mWestToSouth.mInternalLanes, mWestToEast.mInternalLanes, mWestToNorth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandNorthToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mSouthToEast.mInternalLanes, mWestToSouth.mInternalLanes, mWestToEast.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightLefthandNorthToEastTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightLefthandNorthToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionPriorityToRightLefthandNorthToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesWest();
}

lane::LaneIdSet IntersectionPriorityToRightLefthandNorthToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandNorthToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mWestToEast.mInternalLanes, mWestToSouth.mInternalLanes, mWestToNorth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandNorthToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightLefthandNorthToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightLefthandNorthToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionPriorityToRightLefthandNorthToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandNorthToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return getIncomingLanesEast();
}

lane::LaneIdSet IntersectionPriorityToRightLefthandNorthToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToEast.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandNorthToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mEastToWest.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToEast.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightLefthandNorthToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightLefthandFromEastTest::getRouteStart() const
{
  return mFromEast;
}

lane::LaneId IntersectionPriorityToRightLefthandEastToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionPriorityToRightLefthandEastToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesNorth();
}

lane::LaneIdSet IntersectionPriorityToRightLefthandEastToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandEastToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mNorthToWest.mInternalLanes, mNorthToSouth.mInternalLanes, mNorthToEast.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandEastToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mWestToSouth.mInternalLanes, mNorthToWest.mInternalLanes, mNorthToSouth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightLefthandEastToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightLefthandEastToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionPriorityToRightLefthandEastToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesNorth();
}

lane::LaneIdSet IntersectionPriorityToRightLefthandEastToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandEastToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mNorthToSouth.mInternalLanes, mNorthToWest.mInternalLanes, mNorthToEast.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandEastToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mSouthToWest.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightLefthandEastToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightLefthandEastToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionPriorityToRightLefthandEastToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandEastToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return getIncomingLanesSouth();
}

lane::LaneIdSet IntersectionPriorityToRightLefthandEastToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandEastToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightLefthandEastToNorthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightLefthandFromSouthTest::getRouteStart() const
{
  return mFromSouth;
}

lane::LaneId IntersectionPriorityToRightLefthandSouthToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionPriorityToRightLefthandSouthToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesEast();
}

lane::LaneIdSet IntersectionPriorityToRightLefthandSouthToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandSouthToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mEastToNorth.mInternalLanes, mEastToWest.mInternalLanes, mEastToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandSouthToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mNorthToWest.mInternalLanes, mEastToNorth.mInternalLanes, mEastToWest.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightLefthandSouthToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightLefthandSouthToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionPriorityToRightLefthandSouthToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesEast();
}

lane::LaneIdSet IntersectionPriorityToRightLefthandSouthToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandSouthToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mEastToWest.mInternalLanes, mEastToNorth.mInternalLanes, mEastToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandSouthToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightLefthandSouthToNorthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightLefthandSouthToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionPriorityToRightLefthandSouthToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandSouthToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return getIncomingLanesWest();
}

lane::LaneIdSet IntersectionPriorityToRightLefthandSouthToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightLefthandSouthToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mWestToEast.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightLefthandSouthToEastTest, basic_checks)
{
  performBasicChecks();
}

} // namespace map
} // namespace ad
