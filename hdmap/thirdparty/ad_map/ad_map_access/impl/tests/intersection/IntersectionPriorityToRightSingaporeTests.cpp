// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "IntersectionPriorityToRightSingaporeTests.hpp"
#include "MapSetup.hpp"

/*
 * see https://sso.agc.gov.sg/SL/RTA1961-R11?ProvIds=P1IV- for details on special Singapore priority handling here
 */

namespace ad {
namespace map {

void IntersectionPriorityToRightSingaporeTest::prepareMap() const
{
  ::map_setup::prepareMapBasicPriorityToRightSingapore();
}

intersection::IntersectionType IntersectionPriorityToRightSingaporeTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::PriorityToRightAndStraight;
}

lane::LaneId IntersectionPriorityToRightSingaporeFromWestTest::getRouteStart() const
{
  return mFromWest;
}

lane::LaneId IntersectionPriorityToRightSingaporeWestToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeWestToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesSouth();
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeWestToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeWestToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mSouthToEast.mInternalLanes, mSouthToNorth.mInternalLanes, mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeWestToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mEastToNorth.mInternalLanes, mSouthToEast.mInternalLanes, mSouthToNorth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSingaporeWestToNorthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightSingaporeWestToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeWestToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeWestToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesEast(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeWestToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mSouthToNorth.mInternalLanes, mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeWestToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSingaporeWestToEastTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightSingaporeWestToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeWestToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesEast(), getIncomingLanesSouth(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeWestToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeWestToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mSouthToNorth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mNorthToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeWestToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mNorthToSouth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSingaporeWestToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightSingaporeFromNorthTest::getRouteStart() const
{
  return mFromNorth;
}

lane::LaneId IntersectionPriorityToRightSingaporeNorthToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeNorthToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesWest();
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeNorthToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesEast(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeNorthToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mWestToSouth.mInternalLanes, mWestToEast.mInternalLanes, mWestToNorth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeNorthToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mSouthToEast.mInternalLanes, mWestToSouth.mInternalLanes, mWestToEast.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSingaporeNorthToEastTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightSingaporeNorthToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeNorthToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeNorthToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeNorthToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mWestToEast.mInternalLanes, mWestToNorth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeNorthToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mEastToSouth.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSingaporeNorthToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightSingaporeNorthToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeNorthToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesWest(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeNorthToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeNorthToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mEastToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeNorthToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mEastToSouth.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mEastToWest.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSingaporeNorthToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightSingaporeFromEastTest::getRouteStart() const
{
  return mFromEast;
}

lane::LaneId IntersectionPriorityToRightSingaporeEastToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeEastToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesNorth();
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeEastToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeEastToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mNorthToWest.mInternalLanes, mNorthToSouth.mInternalLanes, mNorthToEast.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeEastToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mWestToSouth.mInternalLanes, mNorthToWest.mInternalLanes, mNorthToSouth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSingaporeEastToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightSingaporeEastToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeEastToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeEastToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeEastToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToSouth.mInternalLanes, mNorthToEast.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeEastToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mSouthToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mSouthToNorth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSingaporeEastToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightSingaporeEastToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeEastToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesNorth(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeEastToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeEastToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mSouthToNorth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeEastToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mSouthToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mSouthToNorth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSingaporeEastToNorthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightSingaporeFromSouthTest::getRouteStart() const
{
  return mFromSouth;
}

lane::LaneId IntersectionPriorityToRightSingaporeSouthToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeSouthToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesEast();
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeSouthToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeSouthToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mEastToNorth.mInternalLanes, mEastToWest.mInternalLanes, mEastToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeSouthToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mNorthToWest.mInternalLanes, mEastToNorth.mInternalLanes, mEastToWest.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSingaporeSouthToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightSingaporeSouthToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeSouthToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeSouthToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeSouthToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mEastToWest.mInternalLanes, mEastToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeSouthToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mWestToNorth.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSingaporeSouthToNorthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightSingaporeSouthToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeSouthToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesEast(), getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeSouthToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return lane::LaneIdSet();
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeSouthToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mEastToSouth.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mWestToEast.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSingaporeSouthToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mWestToNorth.mInternalLanes,
                                   mWestToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mWestToEast.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSingaporeSouthToEastTest, basic_checks)
{
  performBasicChecks();
}

} // namespace map
} // namespace ad
