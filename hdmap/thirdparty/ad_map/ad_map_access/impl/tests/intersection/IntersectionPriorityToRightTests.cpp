// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "IntersectionPriorityToRightTests.hpp"
#include "MapSetup.hpp"

namespace ad {
namespace map {

void IntersectionPriorityToRightTest::prepareMap() const
{
  ::map_setup::prepareMapBasicPriorityToRight();
}

intersection::IntersectionType IntersectionPriorityToRightTest::expectedIntersectionType() const
{
  return intersection::IntersectionType::PriorityToRight;
}

lane::LaneId IntersectionPriorityToRightFromWestTest::getRouteStart() const
{
  return mFromWest;
}

lane::LaneId IntersectionPriorityToRightWestToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionPriorityToRightWestToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionPriorityToRightWestToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return getIncomingLanesNorth();
}

lane::LaneIdSet IntersectionPriorityToRightWestToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightWestToNorthTest::expectedCrossingLanes() const
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

TEST_F(IntersectionPriorityToRightWestToNorthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightWestToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionPriorityToRightWestToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesSouth();
}

lane::LaneIdSet IntersectionPriorityToRightWestToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesEast(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionPriorityToRightWestToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mSouthToNorth.mInternalLanes, mSouthToEast.mInternalLanes, mSouthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightWestToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mEastToSouth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightWestToEastTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightWestToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionPriorityToRightWestToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesSouth();
}

lane::LaneIdSet IntersectionPriorityToRightWestToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionPriorityToRightWestToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mSouthToNorth.mInternalLanes, mSouthToWest.mInternalLanes, mSouthToEast.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightWestToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mEastToSouth.mInternalLanes, mNorthToSouth.mInternalLanes, mNorthToEast.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightWestToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightFromNorthTest::getRouteStart() const
{
  return mFromNorth;
}

lane::LaneId IntersectionPriorityToRightNorthToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionPriorityToRightNorthToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesSouth(), getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionPriorityToRightNorthToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return getIncomingLanesEast();
}

lane::LaneIdSet IntersectionPriorityToRightNorthToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightNorthToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mSouthToEast.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightNorthToEastTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightNorthToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionPriorityToRightNorthToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesWest();
}

lane::LaneIdSet IntersectionPriorityToRightNorthToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesEast(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionPriorityToRightNorthToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mWestToNorth.mInternalLanes, mWestToEast.mInternalLanes, mWestToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightNorthToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mEastToSouth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mSouthToWest.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightNorthToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightNorthToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionPriorityToRightNorthToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesWest();
}

lane::LaneIdSet IntersectionPriorityToRightNorthToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesEast(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionPriorityToRightNorthToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mWestToNorth.mInternalLanes, mWestToEast.mInternalLanes, mWestToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightNorthToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mSouthToWest.mInternalLanes, mEastToWest.mInternalLanes, mEastToSouth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightNorthToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightFromEastTest::getRouteStart() const
{
  return mFromEast;
}

lane::LaneId IntersectionPriorityToRightEastToSouthTest::getRouteEnd() const
{
  return mToSouth;
}

lane::LaneIdSet IntersectionPriorityToRightEastToSouthTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesWest()});
}

lane::LaneIdSet IntersectionPriorityToRightEastToSouthTest::expectedIncomingLanesWithLowerPriority() const
{
  return getIncomingLanesSouth();
}

lane::LaneIdSet IntersectionPriorityToRightEastToSouthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightEastToSouthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mSouthToWest.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mWestToSouth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightEastToSouthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightEastToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionPriorityToRightEastToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesNorth();
}

lane::LaneIdSet IntersectionPriorityToRightEastToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionPriorityToRightEastToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mNorthToSouth.mInternalLanes, mNorthToEast.mInternalLanes, mNorthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightEastToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mSouthToWest.mInternalLanes,
                                   mSouthToNorth.mInternalLanes,
                                   mWestToNorth.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToWest.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightEastToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightEastToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionPriorityToRightEastToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesNorth();
}

lane::LaneIdSet IntersectionPriorityToRightEastToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesSouth()});
}

lane::LaneIdSet IntersectionPriorityToRightEastToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mNorthToEast.mInternalLanes, mNorthToSouth.mInternalLanes, mNorthToWest.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightEastToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mWestToNorth.mInternalLanes, mSouthToNorth.mInternalLanes, mSouthToWest.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightEastToNorthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightFromSouthTest::getRouteStart() const
{
  return mFromSouth;
}

lane::LaneId IntersectionPriorityToRightSouthToWestTest::getRouteEnd() const
{
  return mToWest;
}

lane::LaneIdSet IntersectionPriorityToRightSouthToWestTest::expectedIncomingLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesNorth(), getIncomingLanesEast()});
}

lane::LaneIdSet IntersectionPriorityToRightSouthToWestTest::expectedIncomingLanesWithLowerPriority() const
{
  return getIncomingLanesWest();
}

lane::LaneIdSet IntersectionPriorityToRightSouthToWestTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet({mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSouthToWestTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mNorthToSouth.mInternalLanes,
                                   mNorthToWest.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToSouth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSouthToWestTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightSouthToNorthTest::getRouteEnd() const
{
  return mToNorth;
}

lane::LaneIdSet IntersectionPriorityToRightSouthToNorthTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesEast();
}

lane::LaneIdSet IntersectionPriorityToRightSouthToNorthTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionPriorityToRightSouthToNorthTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mEastToNorth.mInternalLanes, mEastToWest.mInternalLanes, mEastToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSouthToNorthTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet({mWestToNorth.mInternalLanes,
                                   mWestToEast.mInternalLanes,
                                   mNorthToEast.mInternalLanes,
                                   mEastToNorth.mInternalLanes,
                                   mEastToWest.mInternalLanes,
                                   mEastToSouth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSouthToNorthTest, basic_checks)
{
  performBasicChecks();
}

lane::LaneId IntersectionPriorityToRightSouthToEastTest::getRouteEnd() const
{
  return mToEast;
}

lane::LaneIdSet IntersectionPriorityToRightSouthToEastTest::expectedIncomingLanesWithHigherPriority() const
{
  return getIncomingLanesEast();
}

lane::LaneIdSet IntersectionPriorityToRightSouthToEastTest::expectedIncomingLanesWithLowerPriority() const
{
  return createUnorderedLaneIdSet({getIncomingLanesWest(), getIncomingLanesNorth()});
}

lane::LaneIdSet IntersectionPriorityToRightSouthToEastTest::expectedInternalLanesWithHigherPriority() const
{
  return createUnorderedLaneIdSet(
    {mEastToNorth.mInternalLanes, mEastToWest.mInternalLanes, mEastToSouth.mInternalLanes});
}

lane::LaneIdSet IntersectionPriorityToRightSouthToEastTest::expectedCrossingLanes() const
{
  return createUnorderedLaneIdSet(
    {mNorthToEast.mInternalLanes, mWestToEast.mInternalLanes, mWestToNorth.mInternalLanes});
}

TEST_F(IntersectionPriorityToRightSouthToEastTest, basic_checks)
{
  performBasicChecks();
}

} // namespace map
} // namespace ad
