// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "SyntheticIntersectionTestBase.hpp"

#include <ad/map/access/Operation.hpp>
#include <algorithm>

// comparison operator for sorting
bool smaller(const ad::map::point::ParaPoint &left, const ad::map::point::ParaPoint &right)
{
  return (left.laneId < right.laneId)
    || ((left.laneId == right.laneId) && (left.parametricOffset < right.parametricOffset));
}

namespace ad {
namespace map {

lane::LaneIdSet SyntheticIntersectionTestBase::getIncomingLanesWithHigherPriority() const
{
  auto resultSet = getIncomingLanes();
  for (auto const &lowerPrioLane : expectedIncomingLanesWithLowerPriority())
  {
    resultSet.erase(lowerPrioLane);
  }
  return resultSet;
}

point::ParaPointList SyntheticIntersectionTestBase::getIncomingParaPointsWithHigherPriority() const
{
  auto resultVector = getIncomingParaPoints();
  auto const incomingLanesWithLowerPriority = getIncomingLanesWithLowerPriority();
  resultVector.erase(std::remove_if(resultVector.begin(),
                                    resultVector.end(),
                                    [&incomingLanesWithLowerPriority](point::ParaPoint const &paraPoint) {
                                      return incomingLanesWithLowerPriority.count(paraPoint.laneId) > 0;
                                    }),
                     resultVector.end());
  return resultVector;
}

lane::LaneIdSet SyntheticIntersectionTestBase::getIncomingLanesWithLowerPriority() const
{
  auto resultSet = getIncomingLanes();
  for (auto const &higherPrioLane : expectedIncomingLanesWithHigherPriority())
  {
    resultSet.erase(higherPrioLane);
  }
  return resultSet;
}

point::ParaPointList SyntheticIntersectionTestBase::getIncomingParaPointsWithLowerPriority() const
{
  auto resultVector = getIncomingParaPoints();
  auto const incomingLanesWithHigherPriority = getIncomingLanesWithHigherPriority();
  resultVector.erase(std::remove_if(resultVector.begin(),
                                    resultVector.end(),
                                    [&incomingLanesWithHigherPriority](point::ParaPoint const &paraPoint) {
                                      return incomingLanesWithHigherPriority.count(paraPoint.laneId) > 0;
                                    }),
                     resultVector.end());
  return resultVector;
}

lane::LaneIdSet SyntheticIntersectionTestBase::getCrossingLanes() const
{
  return expectedCrossingLanes();
}

void SyntheticIntersectionTestBase::compareVectors(point::ParaPointList left, point::ParaPointList right) const
{
  std::sort(left.begin(), left.end(), &smaller);
  std::sort(right.begin(), right.end(), &smaller);
  ASSERT_EQ(left, right);
}

void SyntheticIntersectionTestBase::compareLists(lane::LaneIdSet ids, lane::LaneIdSet otherIds) const
{
  ASSERT_EQ(ids, otherIds);
}

void SyntheticIntersectionTestBase::performBasicChecks()
{
  ASSERT_EQ(expectedIntersectionType(), mIntersection->intersectionType());

  SCOPED_TRACE("internal lanes");
  compareLists(getInternalLanes(), mIntersection->internalLanes());
  SCOPED_TRACE("incoming lanes");
  compareLists(getIncomingLanes(), mIntersection->incomingLanes());
  SCOPED_TRACE("incoming lanes with higher prio");
  compareLists(getIncomingLanesWithHigherPriority(), mIntersection->incomingLanesWithHigherPriority());
  SCOPED_TRACE("incoming lanes with lower prio");
  compareLists(getIncomingLanesWithLowerPriority(), mIntersection->incomingLanesWithLowerPriority());

  compareVectors(getIncomingParaPoints(), mIntersection->incomingParaPoints());
  compareVectors(getIncomingParaPointsWithHigherPriority(), mIntersection->incomingParaPointsWithHigherPriority());
  compareVectors(getIncomingParaPointsWithLowerPriority(), mIntersection->incomingParaPointsWithLowerPriority());
  //@todo verify the geometric problems with the crossing lanes
  if (mIntersection->intersectionType() != intersection::IntersectionType::TrafficLight)
  {
    SCOPED_TRACE("internal lanes with higher prio");
    compareLists(expectedInternalLanesWithHigherPriority(), mIntersection->internalLanesWithHigherPriority());
    SCOPED_TRACE("crossing lanes");
    compareLists(getCrossingLanes(), mIntersection->crossingLanes());
  }
}

TrafficLightForTest SyntheticIntersectionTestBase::expectedTrafficLight(uint64_t landmarkId,
                                                                        landmark::TrafficLightType type) const
{
  TrafficLightForTest expected;
  expected.id = landmark::LandmarkId(landmarkId);
  expected.type = type;
  return expected;
}

TrafficLightForTest SyntheticIntersectionTestBase::expectedTrafficLight(uint64_t landmarkId) const
{
  TrafficLightForTest expected;
  expected.id = landmark::LandmarkId(landmarkId);
  expected.type = landmark::TrafficLightType::UNKNOWN;
  return expected;
}

void SyntheticIntersectionTestBase::performBasicTrafficLightsChecks(
  std::vector<TrafficLightForTest> expectedTrafficLights /* = {}*/)
{
  auto const &trafficLights = mIntersection->applicableTrafficLights();
  ASSERT_EQ(expectedTrafficLights.size(), trafficLights.size()) << "Number of traffic lights don't match";

  for (auto expectedTrafficLight : expectedTrafficLights)
  {
    ASSERT_EQ(1u, trafficLights.count(expectedTrafficLight.id))
      << "Traffic Light ID=" << static_cast<uint64_t>(expectedTrafficLight.id) << "  NOT FOUND";
    if (expectedTrafficLight.type != landmark::TrafficLightType::UNKNOWN)
    {
      ASSERT_EQ(mIntersection->extractTrafficLightType(expectedTrafficLight.id), expectedTrafficLight.type)
        << "Traffic Light type don't match for"
        << "  Traffic Light ID=";
    }
  }

  performBasicChecks();
}

} // namespace map
} // namespace ad
