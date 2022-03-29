// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/lane/LaneOperation.hpp"

#include <algorithm>

#include "LaneOperationPrivate.hpp"
#include "ad/map/access/Operation.hpp"
#include "ad/map/lane/BorderOperation.hpp"
#include "ad/map/match/AdMapMatching.hpp"
#include "ad/map/match/MapMatchedOperation.hpp"
#include "ad/map/point/Operation.hpp"
#include "ad/map/route/LaneIntervalOperation.hpp"
#include "ad/physics/RangeOperation.hpp"

namespace ad {
namespace map {
namespace lane {

Lane::ConstPtr getLanePtr(LaneId const &id) {
  auto const lane = access::getStore().getLanePtr(id);

  if (!bool(lane)) {
    throw std::invalid_argument(
        "ad::map::lane::getLane: LaneId not found in store");
  }
  return lane;
}

Lane const &getLane(LaneId const &id) { return *getLanePtr(id); }

LaneIdList getLanes() { return access::getStore().getLanes(); }

void interpolateHeadingParametricPoints(
    physics::Distance const &length, physics::ParametricValue const &headingT,
    physics::ParametricValue &longTStart, physics::ParametricValue &longTEnd) {
  // we target an area of +- 5cm around the point to determine the heading
  if (length < physics::Distance(0.1)) {
    longTStart = physics::ParametricValue(0.);
    longTEnd = physics::ParametricValue(1.);
  } else {
    physics::ParametricValue deltaOffset(physics::Distance(0.1) / length);
    physics::ParametricValue deltaOffsetHalf = deltaOffset / 2.;
    if (headingT > physics::ParametricValue(1.) - deltaOffsetHalf) {
      longTStart = physics::ParametricValue(1.) - deltaOffset;
      longTEnd = physics::ParametricValue(1.);
    } else if (headingT < deltaOffsetHalf) {
      longTStart = physics::ParametricValue(0.);
      longTEnd = deltaOffset;
    } else {
      longTStart = headingT - deltaOffsetHalf;
      longTEnd = headingT + deltaOffsetHalf;
    }
  }
}

point::ECEFHeading getLaneECEFDirection(Lane const &lane,
                                        point::ParaPoint const &paraPoint) {
  physics::ParametricValue longTStart;
  physics::ParametricValue longTEnd;
  interpolateHeadingParametricPoints(lane.length, paraPoint.parametricOffset,
                                     longTStart, longTEnd);

  point::ECEFPoint const laneDirStart =
      getParametricPoint(lane, longTStart, physics::ParametricValue(.5));
  point::ECEFPoint const laneDirEnd =
      getParametricPoint(lane, longTEnd, physics::ParametricValue(.5));
  point::ECEFHeading const laneDirection =
      point::createECEFHeading(laneDirStart, laneDirEnd);
  return laneDirection;
}

point::ECEFHeading getLaneECEFHeading(
    match::MapMatchedPosition const &mapMatchedPosition) {
  return getLaneECEFHeading(mapMatchedPosition.lanePoint.paraPoint);
}

point::ECEFHeading getLaneECEFHeading(point::ParaPoint const &paraPoint) {
  auto lane = getLane(paraPoint.laneId);

  point::ECEFHeading laneDrivingDirection =
      getLaneECEFDirection(lane, paraPoint);
  if (!isLaneDirectionPositive(lane)) {
    laneDrivingDirection = -1. * laneDrivingDirection;
  }
  return laneDrivingDirection;
}

point::ENUHeading getLaneENUHeading(
    match::MapMatchedPosition const &mapMatchedPosition) {
  return point::createENUHeading(getLaneECEFHeading(mapMatchedPosition),
                                 mapMatchedPosition.matchedPoint);
}

point::ENUHeading getLaneENUHeading(point::ParaPoint const &paraPoint,
                                    point::GeoPoint const &gnssReference) {
  return point::createENUHeading(getLaneECEFHeading(paraPoint), gnssReference);
}

point::ParaPoint uniqueParaPoint(point::GeoPoint const &geoPoint) {
  match::AdMapMatching mapMatching;
  auto mapMatchingResult = mapMatching.getMapMatchedPositions(
      geoPoint, physics::Distance(0.1), physics::Probability(0.5));

  if (mapMatchingResult.size() == 0u) {
    throw std::runtime_error(
        "uniqueLaneId: position doesn't match any lane within 0.1 meters");
  }
  if (mapMatchingResult.size() != 1u) {
    throw std::runtime_error("uniqueLaneId: position matches multiple lanes");
  }

  return mapMatchingResult[0].lanePoint.paraPoint;
}

LaneId uniqueLaneId(point::GeoPoint const &geoPoint) {
  return uniqueParaPoint(geoPoint).laneId;
}

physics::Distance getWidth(Lane const &lane,
                           physics::ParametricValue const &longitudinalOffset) {
  physics::Distance width(0.);
  point::ECEFPoint pointOnLeftEdge;
  point::ECEFPoint pointOnRightEdge;
  if (projectParametricPointToEdges(lane, longitudinalOffset, pointOnLeftEdge,
                                    pointOnRightEdge)) {
    width = distance(pointOnLeftEdge, pointOnRightEdge);
  }
  return width;
}

restriction::SpeedLimitList getSpeedLimits(
    Lane const &lane, physics::ParametricRange const &range) {
  restriction::SpeedLimitList speedLimits;
  for (auto const &speedLimit : lane.speedLimits) {
    if (doRangesOverlap(speedLimit.lanePiece, range)) {
      speedLimits.push_back(speedLimit);
    }
  }
  return speedLimits;
}

physics::Speed getMaxSpeed(Lane const &lane,
                           physics::ParametricRange const &range) {
  physics::Speed maxSpeed(0.);
  for (auto const &speedLimit : getSpeedLimits(lane, range)) {
    maxSpeed = std::max(maxSpeed, speedLimit.speedLimit);
  }
  if (maxSpeed == physics::Speed(0.)) {
    maxSpeed = std::numeric_limits<physics::Speed>::max();
  }
  return maxSpeed;
}

physics::Duration getDuration(Lane const &lane,
                              physics::ParametricRange const &range) {
  physics::Duration laneMinDuration{0.};
  physics::Distance coveredDurationDistance{0.};
  for (auto const &speedLimit : lane.speedLimits) {
    auto const intersectedRange =
        getIntersectionRange(speedLimit.lanePiece, range);
    if (isRangeValid(intersectedRange)) {
      physics::Distance const speedLimitDistance =
          (intersectedRange.maximum - intersectedRange.minimum) * lane.length;
      physics::Speed speedLimitValue =
          std::numeric_limits<physics::Speed>::max();
      if (speedLimit.speedLimit > physics::Speed(0.)) {
        speedLimitValue = speedLimit.speedLimit;
      }
      physics::Duration const speedLimitDuration =
          speedLimitDistance / speedLimitValue;
      laneMinDuration += speedLimitDuration;
      coveredDurationDistance += speedLimitDistance;
    }
  }
  physics::Distance const remainingDistance =
      (range.maximum - range.minimum) * lane.length - coveredDurationDistance;
  if (remainingDistance > physics::Distance(0.)) {
    physics::Duration const speedLimitDuration =
        remainingDistance / getMaxSpeed(lane, range);
    laneMinDuration += speedLimitDuration;
  }
  return laneMinDuration;
}

ContactLaneList getContactLanes(Lane const &lane,
                                ContactLocation const &location) {
  if (location == ContactLocation::INVALID) {
    std::runtime_error("LaneOperation::getContactLanes() location is INVALID");
  }
  ContactLaneList contactLanes;
  for (auto const &contactLane : lane.contactLanes) {
    if (contactLane.location == location) {
      contactLanes.push_back(contactLane);
    }
  }
  return contactLanes;
}

ContactLaneList getContactLanes(Lane const &lane,
                                ContactLocationList const &locations) {
  ContactLaneList contactLanes;
  for (auto const &location : locations) {
    auto const locationContactLanes = getContactLanes(lane, location);
    contactLanes.insert(std::begin(contactLanes),
                        std::begin(locationContactLanes),
                        std::end(locationContactLanes));
  }
  return contactLanes;
}

ContactLocation getContactLocation(Lane const &lane, LaneId const &to_lane_id) {
  if (isValid(to_lane_id)) {
    for (auto const &contact_lane : lane.contactLanes) {
      if (contact_lane.toLane == to_lane_id) {
        return contact_lane.location;
      }
    }
  }
  return ContactLocation::INVALID;
}

point::ECEFPoint getParametricPoint(
    Lane const &lane, physics::ParametricValue const &longitudinalOffset,
    physics::ParametricValue const &lateralOffset) {
  point::ECEFPoint const pt0 =
      getParametricPoint(lane.edgeLeft, longitudinalOffset);
  if (isValid(pt0)) {
    point::ECEFPoint const pt1 =
        getParametricPoint(lane.edgeRight, longitudinalOffset);
    if (isValid(pt1)) {
      return point::vectorInterpolate(pt0, pt1, lateralOffset);
    }
  }
  return point::ECEFPoint();
}

bool projectParametricPointToEdges(Lane const &lane,
                                   point::ECEFPoint const &referencePoint,
                                   point::ECEFPoint &pointOnLeftEdge,
                                   point::ECEFPoint &pointOnRightEdge) {
  if (isValid(referencePoint)) {
    auto const longTLeft =
        findNearestPointOnEdge(lane.edgeLeft, referencePoint);
    if (isRangeValid(longTLeft)) {
      auto const longTRight =
          findNearestPointOnEdge(lane.edgeRight, referencePoint);
      if (isRangeValid(longTRight)) {
        pointOnLeftEdge = getParametricPoint(lane.edgeLeft, longTLeft);
        pointOnRightEdge = getParametricPoint(lane.edgeRight, longTRight);
        return isValid(pointOnLeftEdge) && isValid(pointOnRightEdge);
      }
    }
  }
  return false;
}

bool projectParametricPointToEdges(
    Lane const &lane, physics::ParametricValue const &longitudinalOffset,
    point::ECEFPoint &pointOnLeftEdge, point::ECEFPoint &pointOnRightEdge) {
  point::ECEFPoint const centerPoint = getParametricPoint(
      lane, longitudinalOffset, physics::ParametricValue(0.5));
  return projectParametricPointToEdges(lane, centerPoint, pointOnLeftEdge,
                                       pointOnRightEdge);
}

point::ECEFPoint getProjectedParametricPoint(
    Lane const &lane, physics::ParametricValue const &longitudinalOffset,
    physics::ParametricValue const &lateralOffset) {
  point::ECEFPoint pt0;
  point::ECEFPoint pt1;
  bool projectionResult =
      projectParametricPointToEdges(lane, longitudinalOffset, pt0, pt1);
  if (projectionResult) {
    return point::vectorInterpolate(pt0, pt1, lateralOffset);
  }
  return point::ECEFPoint();
}

bool isPyhsicalSuccessor(Lane const &lane, const Lane &other) {
  if (isSuccessor(lane.edgeLeft, other.edgeLeft) &&
      isSuccessor(lane.edgeRight, other.edgeRight)) {
    return true;
  }
  if (isSuccessor(lane.edgeLeft, other.edgeRight) &&
      isSuccessor(lane.edgeRight, other.edgeLeft)) {
    return true;
  }
  if (isVanishingLaneStart(lane) || isVanishingLaneEnd(lane)) {
    if (isSuccessor(lane.edgeLeft, other.edgeLeft) &&
        isSuccessor(lane.edgeRight, other.edgeLeft)) {
      return true;
    }
    if (isSuccessor(lane.edgeLeft, other.edgeRight) &&
        isSuccessor(lane.edgeRight, other.edgeRight)) {
      return true;
    }
  }
  if (isVanishingLaneEnd(other)) {
    if (isSuccessor(lane.edgeLeft, other.edgeRight) ||
        isSuccessor(lane.edgeRight, other.edgeRight)) {
      return true;
    }
  }
  return false;
}

bool isPhysicalPredecessor(Lane const &lane, const Lane &other) {
  if (isPredecessor(lane.edgeLeft, other.edgeLeft) &&
      isPredecessor(lane.edgeRight, other.edgeRight)) {
    return true;
  }
  if (isPredecessor(lane.edgeLeft, other.edgeRight) &&
      isPredecessor(lane.edgeRight, other.edgeLeft)) {
    return true;
  }
  if (isVanishingLaneStart(lane) || isVanishingLaneEnd(lane)) {
    if (isPredecessor(lane.edgeLeft, other.edgeLeft) &&
        isPredecessor(lane.edgeRight, other.edgeLeft)) {
      return true;
    }
    if (isPredecessor(lane.edgeLeft, other.edgeRight) &&
        isPredecessor(lane.edgeRight, other.edgeRight)) {
      return true;
    }
  }
  if (isVanishingLaneStart(other)) {
    if (isPredecessor(lane.edgeLeft, other.edgeRight) ||
        isPredecessor(lane.edgeRight, other.edgeRight)) {
      return true;
    }
  }
  return false;
}

bool isVanishingLaneStart(Lane const &lane) {
  return haveSameStart(lane.edgeLeft, lane.edgeRight);
}

bool isVanishingLaneEnd(Lane const &lane) {
  return haveSameEnd(lane.edgeLeft, lane.edgeRight);
}

bool satisfiesFilter(Lane const &lane, std::string const &typeFilter,
                     bool isHov) {
  if (isHov == (getHOV(lane) > restriction::PassengerCount(1))) {
    if (!typeFilter.empty()) {
      auto const laneFullTypeString = toString(lane.type);
      if (typeFilter.find(laneFullTypeString) != std::string::npos) {
        return true;
      }
      std::size_t found = laneFullTypeString.find_last_of(":");
      auto const laneTypeWithoutPrefix = laneFullTypeString.substr(found + 1);
      if (!laneTypeWithoutPrefix.empty()) {
        if (typeFilter.find(laneTypeWithoutPrefix) != std::string::npos) {
          return true;
        }
      }
    } else {
      return true;
    }
  }
  return false;
}

void updateLaneLengths(Lane &lane) {
  if (isValid(lane.edgeLeft)) {
    if (isValid(lane.edgeRight)) {
      lane.lengthRange.minimum =
          std::min(lane.edgeLeft.length, lane.edgeRight.length);
      lane.lengthRange.maximum =
          std::max(lane.edgeLeft.length, lane.edgeRight.length);
      lane.length = (lane.edgeLeft.length + lane.edgeRight.length) * 0.5;

      auto widthRangeResult =
          calculateWidthRange(lane.edgeLeft.ecefEdge, lane.edgeLeft.length,
                              lane.edgeRight.ecefEdge, lane.edgeRight.length);
      lane.widthRange = widthRangeResult.first;
      lane.width = widthRangeResult.second;
    } else {
      lane.length = lane.edgeLeft.length;
      lane.lengthRange.minimum = lane.length;
      lane.lengthRange.maximum = lane.length;
      lane.width = physics::Distance(0);
      lane.widthRange.minimum = lane.width;
      lane.widthRange.maximum = lane.width;
    }
  } else if (isValid(lane.edgeRight)) {
    lane.length = lane.edgeRight.length;
    lane.lengthRange.minimum = lane.length;
    lane.lengthRange.maximum = lane.length;
    lane.width = physics::Distance(0);
    lane.widthRange.minimum = lane.width;
    lane.widthRange.maximum = lane.width;
  } else {
    lane.length = physics::Distance(0);
    lane.lengthRange.minimum = lane.length;
    lane.lengthRange.maximum = lane.length;
    lane.width = physics::Distance(0);
    lane.widthRange.minimum = lane.width;
    lane.widthRange.maximum = lane.width;
  }
}

point::ENUHeading getLaneENUHeading(point::ParaPoint const &position) {
  const auto gnssReference = access::getENUReferencePoint();
  const auto laneHeading = getLaneENUHeading(position, gnssReference);
  return laneHeading;
}

bool isHeadingInLaneDirection(point::ParaPoint const &position,
                              point::ENUHeading const &heading) {
  const auto laneHeading = getLaneENUHeading(position);
  // enforce normalization of angle difference
  const auto headingDifference = point::createENUHeading(
      static_cast<double>(std::fabs(heading - laneHeading)));

  if (static_cast<double>(std::fabs(headingDifference)) > M_PI / 2.) {
    return false;
  }

  return true;
}

bool isLaneDirectionPositive(LaneId const &laneId) {
  auto const lane = getLane(laneId);
  return isLaneDirectionPositive(lane);
}

bool isLaneDirectionNegative(LaneId const &laneId) {
  auto const lane = getLane(laneId);
  return isLaneDirectionNegative(lane);
}

bool projectPositionToLaneInHeadingDirection(
    point::ParaPoint const &position, point::ENUHeading const &heading,
    point::ParaPoint &projectedPosition) {
  projectedPosition = position;
  if (isHeadingInLaneDirection(position, heading)) {
    return true;
  }

  const auto lane = getLane(position.laneId);
  ContactLocationList types;
  if (isLaneDirectionPositive(position.laneId)) {
    types.push_back(ContactLocation::LEFT);
    types.push_back(ContactLocation::RIGHT);
  } else {
    types.push_back(ContactLocation::RIGHT);
    types.push_back(ContactLocation::LEFT);
  }
  for (auto contactType : types) {
    auto contactLanes = getContactLanes(lane, contactType);
    while (contactLanes.size() > 0) {
      // check first neighbor
      const auto neighborLaneId = contactLanes[0].toLane;
      const auto neighborLane = getLane(neighborLaneId);

      projectedPosition.laneId = neighborLaneId;
      projectedPosition.parametricOffset = position.parametricOffset;

      if (isRouteable(neighborLane) &&
          isHeadingInLaneDirection(projectedPosition, heading)) {
        return true;
      } else {
        contactLanes = getContactLanes(neighborLane, contactType);
      }
    }
  }

  return false;
}

match::MapMatchedPosition calcMapMatchedPosition(
    Lane const &lane, physics::ParametricValue const &longTLeft,
    physics::ParametricValue const &longTRight, point::ECEFPoint const &pt) {
  match::MapMatchedPosition mmpos;
  point::ECEFPoint const pt_left =
      point::getParametricPoint(lane.edgeLeft, longTLeft);
  point::ECEFPoint const pt_right =
      point::getParametricPoint(lane.edgeRight, longTRight);

  mmpos.lanePoint.paraPoint.laneId = lane.id;
  mmpos.lanePoint.lateralT =
      point::findNearestPointOnEdge(pt, pt_left, pt_right);
  physics::ParametricValue nearestT;
  if (mmpos.lanePoint.lateralT < physics::RatioValue(0.)) {
    nearestT = physics::ParametricValue(0.);
    mmpos.type = match::MapMatchedPositionType::LANE_LEFT;
    mmpos.probability = std::max(
        physics::Probability(0.1),
        physics::Probability(
            0.5 + static_cast<double>(mmpos.lanePoint.lateralT) / 10.));
  } else if (mmpos.lanePoint.lateralT > physics::RatioValue(1.)) {
    nearestT = physics::ParametricValue(1.);
    mmpos.type = match::MapMatchedPositionType::LANE_RIGHT;
    mmpos.probability = std::max(
        physics::Probability(0.1),
        physics::Probability(
            0.5 - (static_cast<double>(mmpos.lanePoint.lateralT) - 1.) / 10.));
  } else {
    nearestT =
        physics::ParametricValue(static_cast<double>(mmpos.lanePoint.lateralT));
    mmpos.type = match::MapMatchedPositionType::LANE_IN;
    mmpos.probability =
        physics::Probability(1.) -
        std::min(physics::Probability(0.5),
                 physics::Probability(fabs(
                     0.5 - static_cast<double>(mmpos.lanePoint.lateralT))));
  }
  mmpos.matchedPoint = point::vectorInterpolate(pt_left, pt_right, nearestT);
  mmpos.lanePoint.paraPoint.parametricOffset =
      nearestT * longTLeft +
      (physics::ParametricValue(1.) - nearestT) * longTRight;
  mmpos.lanePoint.laneLength = lane.length;
  mmpos.lanePoint.laneWidth = point::distance(pt_left, pt_right);
  mmpos.queryPoint = pt;
  mmpos.matchedPointDistance =
      point::distance(mmpos.matchedPoint, mmpos.queryPoint);
  return mmpos;
}

bool findNearestPointOnLane(Lane const &lane, point::ECEFPoint const &pt,
                            match::MapMatchedPosition &mmpos) {
  auto const longTLeft = findNearestPointOnEdge(lane.edgeLeft, pt);
  if (longTLeft.isValid()) {
    auto const longTRight = findNearestPointOnEdge(lane.edgeRight, pt);
    if (longTRight.isValid()) {
      mmpos = calcMapMatchedPosition(lane, longTLeft, longTRight, pt);
      return true;
    }
  }
  return false;
}

bool findNearestPointOnLaneInterval(route::LaneInterval const &laneInterval,
                                    point::ECEFPoint const &pt,
                                    match::MapMatchedPosition &mmpos) {
  auto const &lane = getLane(laneInterval.laneId);
  auto const laneRange = route::toParametricRange(laneInterval);
  auto longTLeft = findNearestPointOnEdge(lane.edgeLeft, pt);
  if (longTLeft.isValid()) {
    auto longTRight = findNearestPointOnEdge(lane.edgeRight, pt);
    if (longTRight.isValid()) {
      if (!physics::isWithinRange(laneRange, longTLeft)) {
        if (longTLeft < laneRange.minimum) {
          longTLeft = laneRange.minimum;
        } else if (longTLeft > laneRange.maximum) {
          longTLeft = laneRange.maximum;
        }
      }
      if (!physics::isWithinRange(laneRange, longTRight)) {
        if (longTRight < laneRange.minimum) {
          longTRight = laneRange.minimum;
        } else if (longTRight > laneRange.maximum) {
          longTRight = laneRange.maximum;
        }
      }

      mmpos = calcMapMatchedPosition(lane, longTLeft, longTRight, pt);
      return true;
    }
  }
  return false;
}

physics::Distance getDistanceToLane(LaneId laneId,
                                    match::Object const &object) {
  // prefer fast checks first
  for (auto const &occupiedLane :
       object.mapMatchedBoundingBox.laneOccupiedRegions) {
    if (occupiedLane.laneId == laneId) {
      return physics::Distance(0.);
    }
  }
  // already map matching result available, but not within the lane
  physics::Distance distance = std::numeric_limits<physics::Distance>::max();
  bool mapMatchedPointFound = false;
  point::ECEFPoint const objectPointECEF = point::toECEF(
      object.enuPosition.centerPoint, object.enuPosition.enuReferencePoint);

  for (auto referencePoint : {match::ObjectReferencePoints::FrontLeft,
                              match::ObjectReferencePoints::FrontRight,
                              match::ObjectReferencePoints::RearLeft,
                              match::ObjectReferencePoints::RearRight}) {
    auto mapMatchedConfidenceList =
        object.mapMatchedBoundingBox
            .referencePointPositions[size_t(referencePoint)];
    for (auto const &mapMatchedPosition : mapMatchedConfidenceList) {
      if (mapMatchedPosition.lanePoint.paraPoint.laneId == laneId) {
        mapMatchedPointFound = true;
        // use the smallest distance if multiple have been matched
        distance = std::min(
            distance,
            point::distance(mapMatchedPosition.matchedPoint, objectPointECEF));
      }
    }
  }
  if (!mapMatchedPointFound) {
    // need to perform map matching on our own
    auto lane = getLane(laneId);
    match::MapMatchedPosition mapMatchedPosition;
    if (findNearestPointOnLane(lane, objectPointECEF, mapMatchedPosition)) {
      distance =
          point::distance(mapMatchedPosition.matchedPoint, objectPointECEF);
      distance =
          std::max(distance - std::max(object.enuPosition.dimension.length,
                                       object.enuPosition.dimension.width) /
                                  2.,
                   physics::Distance(0.));
    }
  }

  return distance;
}

physics::Distance calcLength(LaneId const &laneId) {
  const auto lane = getLanePtr(laneId);
  return lane->length;
}

physics::Distance calcLength(
    match::LaneOccupiedRegion const &laneOccupiedRegion) {
  auto const rangeLength = (laneOccupiedRegion.longitudinalRange.maximum -
                            laneOccupiedRegion.longitudinalRange.minimum);
  return rangeLength * calcLength(laneOccupiedRegion.laneId);
}

physics::Distance calcWidth(point::ParaPoint const &paraPoint) {
  return calcWidth(paraPoint.laneId, paraPoint.parametricOffset);
}

physics::Distance calcWidth(LaneId const &laneId,
                            physics::ParametricValue const &longOffset) {
  const auto lane = getLanePtr(laneId);
  return getWidth(*lane, longOffset);
}

physics::Distance calcWidth(point::ENUPoint const &enuPoint) {
  match::AdMapMatching mapMatching;
  const auto mapMatchedPositions = mapMatching.getMapMatchedPositions(
      enuPoint, physics::Distance(1.0), physics::Probability(0.1));

  if (mapMatchedPositions.empty()) {
    return physics::Distance(-1.0);
  }
  return calcWidth(mapMatchedPositions.front().lanePoint.paraPoint);
}

physics::Distance calcWidth(
    match::LaneOccupiedRegion const &laneOccupiedRegion) {
  auto const rangeWidth = (laneOccupiedRegion.lateralRange.maximum -
                           laneOccupiedRegion.lateralRange.minimum);
  return rangeWidth * calcWidth(match::getCenterParaPoint(laneOccupiedRegion));
}

ContactLocation getDirectNeighborhoodRelation(LaneId const laneId,
                                              LaneId const checkLaneId) {
  if (laneId == checkLaneId) {
    return ContactLocation::OVERLAP;
  }
  auto lane = getLane(laneId);
  for (auto contactLocation :
       {ContactLocation::LEFT, ContactLocation::RIGHT,
        ContactLocation::SUCCESSOR, ContactLocation::PREDECESSOR}) {
    auto contactLanes = getContactLanes(lane, contactLocation);
    auto findResult =
        std::find_if(std::begin(contactLanes), std::end(contactLanes),
                     [checkLaneId](ContactLane const &contactLane) {
                       return contactLane.toLane == checkLaneId;
                     });
    if (findResult != std::end(contactLanes)) {
      return contactLocation;
    }
  }
  return ContactLocation::INVALID;
}

bool isSuccessorOrPredecessor(LaneId const laneId, LaneId const checkLaneId) {
  auto const neighborhood = getDirectNeighborhoodRelation(laneId, checkLaneId);
  return (neighborhood == ContactLocation::SUCCESSOR) ||
         (neighborhood == ContactLocation::PREDECESSOR);
}

bool isSameOrDirectNeighbor(LaneId const laneId, LaneId const checkLaneId) {
  auto const neighborhood = getDirectNeighborhoodRelation(laneId, checkLaneId);
  return (neighborhood == ContactLocation::OVERLAP) ||
         (neighborhood == ContactLocation::LEFT) ||
         (neighborhood == ContactLocation::RIGHT);
}

point::ENUPoint getENULanePoint(point::ParaPoint const parametricPoint,
                                physics::ParametricValue const &lateralOffset) {
  // perform map matching
  auto lane = getLane(parametricPoint.laneId);
  auto ecefPoint =
      getParametricPoint(lane, parametricPoint.parametricOffset, lateralOffset);
  auto enuPoint = point::toENU(ecefPoint);
  return enuPoint;
}
}  // namespace lane
}  // namespace map
}  // namespace ad
