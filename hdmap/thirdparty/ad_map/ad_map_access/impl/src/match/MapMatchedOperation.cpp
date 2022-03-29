// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/match/MapMatchedOperation.hpp"

#include <algorithm>
#include "ad/map/point/Operation.hpp"

namespace ad {
namespace map {
namespace match {

point::ParaPointList getParaPoints(MapMatchedPositionConfidenceList const &inMapMatchedPositions)
{
  point::ParaPointList result;
  result.reserve(inMapMatchedPositions.size());
  for (auto const &mapMatchedPosition : inMapMatchedPositions)
  {
    result.push_back(mapMatchedPosition.lanePoint.paraPoint);
  }
  return result;
}

point::ENUHeading getObjectENUHeading(const match::MapMatchedObjectBoundingBox &mapMatchedBoundingBox)
{
  point::ECEFPoint rearLeftPosition;
  bool const rearLeftAvailable
    = mapMatchedBoundingBox.referencePointPositions[int32_t(match::ObjectReferencePoints::RearLeft)].size() > 0u;
  if (rearLeftAvailable)
  {
    rearLeftPosition = mapMatchedBoundingBox.referencePointPositions[int32_t(match::ObjectReferencePoints::RearLeft)]
                         .front()
                         .queryPoint;
  }
  point::ECEFPoint rearRightPosition;
  bool const rearRightAvailable
    = mapMatchedBoundingBox.referencePointPositions[int32_t(match::ObjectReferencePoints::RearRight)].size() > 0u;
  if (rearRightAvailable)
  {
    rearRightPosition = mapMatchedBoundingBox.referencePointPositions[int32_t(match::ObjectReferencePoints::RearRight)]
                          .front()
                          .queryPoint;
  }
  point::ECEFPoint frontLeftPosition;
  bool const frontLeftAvailable
    = mapMatchedBoundingBox.referencePointPositions[int32_t(match::ObjectReferencePoints::FrontLeft)].size() > 0u;
  if (frontLeftAvailable)
  {
    frontLeftPosition = mapMatchedBoundingBox.referencePointPositions[int32_t(match::ObjectReferencePoints::FrontLeft)]
                          .front()
                          .queryPoint;
  }
  point::ECEFPoint frontRightPosition;
  bool const frontRightAvailable
    = mapMatchedBoundingBox.referencePointPositions[int32_t(match::ObjectReferencePoints::FrontRight)].size() > 0u;
  if (frontRightAvailable)
  {
    frontRightPosition
      = mapMatchedBoundingBox.referencePointPositions[int32_t(match::ObjectReferencePoints::FrontRight)]
          .front()
          .queryPoint;
  }

  point::ECEFHeading ecefHeading;
  bool rotateENUHeading = false;
  if (rearLeftAvailable && rearRightAvailable && frontLeftAvailable && frontRightAvailable)
  {
    auto const rearCenterPosition = 0.5 * (rearLeftPosition + rearRightPosition);
    auto const frontCenterPosition = 0.5 * (frontLeftPosition + frontRightPosition);
    ecefHeading = point::createECEFHeading(rearCenterPosition, frontCenterPosition);
  }
  else if (rearLeftAvailable && frontLeftAvailable)
  {
    ecefHeading = point::createECEFHeading(rearLeftPosition, frontLeftPosition);
  }
  else if (rearRightAvailable && frontRightAvailable)
  {
    ecefHeading = point::createECEFHeading(rearRightPosition, frontRightPosition);
  }
  else if (frontLeftAvailable && frontRightAvailable)
  {
    ecefHeading = point::createECEFHeading(frontLeftPosition, frontRightPosition);
    rotateENUHeading = true;
  }
  else if (rearLeftAvailable && rearRightAvailable)
  {
    ecefHeading = point::createECEFHeading(rearLeftPosition, rearRightPosition);
    rotateENUHeading = true;
  }
  else
  {
    throw std::runtime_error("point::createENUHeading no two corner points available. Heading estimate is impossible");
  }
  auto enuHeading = point::createENUHeading(ecefHeading);
  if (rotateENUHeading)
  {
    enuHeading = point::createENUHeading(static_cast<double>(enuHeading) + M_PI_2);
  }
  return enuHeading;
}

physics::Distance signedDistanceToLane(lane::LaneId const checkLaneId,
                                       MapMatchedPositionConfidenceList const &mapMatchedPositions)
{
  physics::Distance distance = std::numeric_limits<physics::Distance>::max();

  auto findCheckPosition = std::find_if(std::begin(mapMatchedPositions),
                                        std::end(mapMatchedPositions),
                                        [checkLaneId](match::MapMatchedPosition const &position) {
                                          return position.lanePoint.paraPoint.laneId == checkLaneId;
                                        });

  if (findCheckPosition != std::end(mapMatchedPositions))
  {
    // found position
    if (findCheckPosition->type == match::MapMatchedPositionType::LANE_IN)
    {
      distance = physics::Distance(0.);
    }
    else if (findCheckPosition->type == match::MapMatchedPositionType::LANE_LEFT)
    {
      if (findCheckPosition->lanePoint.lateralT > physics::RatioValue(0.))
      {
        throw std::runtime_error("ad::map::match::signedDistanceToLane: inconsistent map matched positions."
                                 " Expected lateralT to be < 0. for LANE_LEFT");
      }
      distance = findCheckPosition->lanePoint.lateralT * findCheckPosition->lanePoint.laneWidth;
    }
    else if (findCheckPosition->type == match::MapMatchedPositionType::LANE_RIGHT)
    {
      if (findCheckPosition->lanePoint.lateralT < physics::RatioValue(1.))
      {
        throw std::runtime_error("ad::map::match::signedDistanceToLane: inconsistent map matched positions."
                                 " Expected lateralT to be > 1. for LANE_RIGHT");
      }
      distance
        = (findCheckPosition->lanePoint.lateralT - physics::RatioValue(1.)) * findCheckPosition->lanePoint.laneWidth;
    }
    else
    {
      throw std::runtime_error("ad::map::match::signedDistanceToLane: inconsistent map matched positions."
                               " Expected type to be one of LANE_IN, LANE_LEFT, LEFT_RIGHT");
    }
  }

  return distance;
}

} // namespace match
} // namespace map
} // namespace ad
