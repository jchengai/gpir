// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2019-2020 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/opendrive/AdMapFactory.hpp"

#include <boost/algorithm/string.hpp>
#include <opendrive/OpenDrive.hpp>

#include "DataTypeConversion.hpp"
#include "ad/map/access/Logging.hpp"
#include "ad/map/point/GeoOperation.hpp"
#include "ad/map/point/Transform.hpp"

namespace ad {
namespace map {
namespace opendrive {

AdMapFactory::AdMapFactory(access::Store &store) : access::Factory(store) {}

bool AdMapFactory::isOpenDriveMap(std::string const &mapName) {
  // @todo Check whether the file is xml and contains the tags <OpenDRIVE>
  return ::boost::iends_with(mapName, ".xodr");
}

bool AdMapFactory::createAdMap(
    std::string const &mapFilePath, double const overlapMargin,
    intersection::IntersectionType const defaultIntersectionType,
    landmark::TrafficLightType const defaultTrafficLightType) {
  // parse data from xml file
  ::opendrive::OpenDriveData openDriveData;
  if (!::opendrive::Load(mapFilePath, openDriveData)) {
    access::getLogger()->warn("Unable to open opendrive map for reading {}",
                              mapFilePath);
    return false;
  }
  return createAdMap(openDriveData, overlapMargin, defaultIntersectionType,
                     defaultTrafficLightType);
}

bool AdMapFactory::createAdMapFromString(
    std::string const &mapContent, double const overlapMargin,
    intersection::IntersectionType const defaultIntersectionType,
    landmark::TrafficLightType const defaultTrafficLightType) {
  // parse data from string
  ::opendrive::OpenDriveData openDriveData;
  if (!::opendrive::Parse(mapContent, openDriveData)) {
    access::getLogger()->warn("Unable to parse opendrive content");
    return false;
  }
  return createAdMap(openDriveData, overlapMargin, defaultIntersectionType,
                     defaultTrafficLightType);
}

bool AdMapFactory::createAdMap(
    ::opendrive::OpenDriveData &openDriveData, double const overlapMargin,
    intersection::IntersectionType const defaultIntersectionType,
    landmark::TrafficLightType const defaultTrafficLightType) {
  if (!::opendrive::GenerateLaneMap(openDriveData, overlapMargin)) {
    access::getLogger()->warn("LaneMap geometry generated with errors");
  }

  access::getLogger()->info("Opened opendrive map ");

  if (std::isnan(openDriveData.geoReference.latitude) ||
      std::isnan(openDriveData.geoReference.longitude)) {
    auto geoRefPoint = access::getENUReferencePoint();
    openDriveData.geoReference.latitude =
        static_cast<double>(geoRefPoint.latitude);
    openDriveData.geoReference.longitude =
        static_cast<double>(geoRefPoint.longitude);
  } else {
    point::GeoPoint geoRefPoint;
    geoRefPoint.longitude =
        ::ad::map::point::Longitude(openDriveData.geoReference.longitude);
    geoRefPoint.latitude =
        ::ad::map::point::Latitude(openDriveData.geoReference.latitude);
    geoRefPoint.altitude =
        ::ad::map::point::Altitude(openDriveData.geoReference.altitude);
    access::setENUReferencePoint(geoRefPoint);
  }

  access::getLogger()->info("Using reference point {}",
                            access::getENUReferencePoint());

  return convertToAdMap(openDriveData, defaultIntersectionType,
                        defaultTrafficLightType);
}

restriction::Restrictions AdMapFactory::createRoadRestrictions() const {
  restriction::Restriction roadRestriction;
  roadRestriction.negated = false;
  roadRestriction.passengersMin = 0.;
  roadRestriction.roadUserTypes = {restriction::RoadUserType::CAR,
                                   restriction::RoadUserType::BUS,
                                   restriction::RoadUserType::TRUCK};
  restriction::Restrictions roadRestrictions;
  roadRestrictions.conjunctions.push_back(roadRestriction);
  return roadRestrictions;
}

bool AdMapFactory::setLaneSpeed(::opendrive::Lane const &lane) {
  bool ok = true;

  auto laneId = toLaneId(lane.id);

  for (auto const &parametricSpeed : lane.speed) {
    restriction::SpeedLimit speedLimit;
    speedLimit.lanePiece.minimum =
        physics::ParametricValue(parametricSpeed.start);
    speedLimit.lanePiece.maximum =
        physics::ParametricValue(parametricSpeed.end);
    speedLimit.speedLimit = physics::Speed(parametricSpeed.speed);
    if (!add(laneId, speedLimit)) {
      ok = false;
    }
  }

  return ok;
}

bool AdMapFactory::addLandmark(::opendrive::Landmark const &landmark) {
  bool ok = true;

  point::Geometry boundingBox;
  landmark::LandmarkId landmarkId(toLandmarkId(landmark.id));
  access::PartitionId partitionId(0);

  auto position = toECEF(landmark.position);

  point::Longitude landmarkDirectionLon(landmark.position.x +
                                        cos(landmark.orientation));
  point::Latitude landmarkDirectionLat(landmark.position.y +
                                       sin(landmark.orientation));

  point::GeoPoint geoOrientation = point::createGeoPoint(
      landmarkDirectionLon, landmarkDirectionLat, point::Altitude(0.));
  auto orientation = point::toECEF(geoOrientation);

  auto landmarkType = toLandmarkType(landmark.type);

  if (landmarkType == landmark::LandmarkType::TRAFFIC_LIGHT) {
    auto trafficLightType = toTrafficLightType(landmark.type, landmark.subtype);
    if (!addTrafficLight(partitionId, landmarkId, trafficLightType, position,
                         orientation, boundingBox)) {
      ok = false;
    }
  } else if (landmarkType == landmark::LandmarkType::TRAFFIC_SIGN) {
    auto trafficSignType = toTrafficSignType(landmark.type, landmark.subtype);
    if (!addTrafficSign(partitionId, landmarkId, trafficSignType, position,
                        orientation, boundingBox, "None")) {
      ok = false;
    }
  } else {
    if (Factory::addLandmark(partitionId, landmarkId, landmarkType, position,
                             orientation, boundingBox)) {
      ok = false;
    }
  }

  return ok;
}

bool AdMapFactory::isDrivableLane(lane::LaneType laneType) const {
  switch (laneType) {
    case lane::LaneType::NORMAL:
    case lane::LaneType::MULTI:
    case lane::LaneType::TURN:
    case lane::LaneType::INTERSECTION:
      return true;
    default:
      return false;
  }
}

bool AdMapFactory::addLane(::opendrive::Lane const &lane) {
  bool ok = true;
  if ((lane.leftEdge.size() < 2) || (lane.rightEdge.size() < 2)) {
    // Invalid number of points for lane"
    access::getLogger()->error("Invalid number of points for lane {}", lane.id);
    return false;
  }

  lane::LaneType type = toLaneType(lane.type);

  // opendrive assigns an id if the lane belongs to a certain junction
  if ((lane.junction != -1) && isDrivableLane(type)) {
    type = lane::LaneType::INTERSECTION;
  }

  lane::LaneDirection direction = toLaneDirection(lane);

  auto partitionId = access::PartitionId(0);
  auto laneId = toLaneId(lane.id);

  if (!add(partitionId, laneId, type, direction)) {
    ok = false;
  }
  auto const leftEcefEdge = toGeometry(lane.leftEdge);
  auto const rightEcefEdge = toGeometry(lane.rightEdge);
  if (!set(laneId, leftEcefEdge, rightEcefEdge)) {
    ok = false;
  }
  if (!setLaneSpeed(lane)) {
    ok = false;
  }

  return ok;
}

bool AdMapFactory::addSpecialContact(::opendrive::Lane const &lane,
                                     ::opendrive::Landmark const &landmark,
                                     lane::ContactLocation const &location) {
  bool ok = true;
  auto const contact = ::ad::map::opendrive::toContactType(landmark.type);
  if ((contact == lane::ContactType::UNKNOWN)) {
    // The landmark does not generate any contact
    return true;
  } else if (contact == lane::ContactType::INVALID) {
    access::getLogger()->warn("addSpecialContact() Invalid contact type");
    return false;
  }

  auto const contacts = lane::ContactTypeList{contact};
  auto const restrictions =
      createRoadRestrictions();  // to do generate restrictions from traffic
                                 // signs
  auto const &laneContacts = (location == lane::ContactLocation::SUCCESSOR)
                                 ? lane.successors
                                 : lane.predecessors;

  for (auto const &contactId : laneContacts) {
    if (contact == lane::ContactType::TRAFFIC_LIGHT) {
      landmark::LandmarkId trafficLightId(toLandmarkId(landmark.id));
      if (!add(toLaneId(lane.id), toLaneId(contactId), location, contacts,
               restrictions, trafficLightId)) {
        ok = false;
      }
    } else {
      if (!add(toLaneId(lane.id), toLaneId(contactId), location, contacts,
               restrictions)) {
        ok = false;
      }
    }
  }
  return ok;
}

bool AdMapFactory::addSpecialContacts(
    ::opendrive::Lane const &lane, ::opendrive::LandmarkMap const &landmarks) {
  bool ok = true;
  for (auto const &signalReference : lane.signalReferences) {
    auto const location =
        toContactLocation(signalReference, lane.junction != -1);
    auto const &landmark = landmarks.at(signalReference.id);
    if (!addSpecialContact(lane, landmark, location)) {
      ok = false;
    }
  }

  return ok;
}

bool AdMapFactory::addContactLanes(
    ::opendrive::Lane const &lane,
    intersection::IntersectionType const defaultIntersectionType,
    landmark::TrafficLightType const defaultTrafficLightType) {
  bool ok = true;

  lane::ContactTypeList const laneContinuation(
      {lane::ContactType::LANE_CONTINUATION});
  lane::ContactTypeList const laneChange({lane::ContactType::LANE_CHANGE});
  lane::ContactLocation const left(lane::ContactLocation::LEFT);
  lane::ContactLocation const right(lane::ContactLocation::RIGHT);
  lane::ContactLocation const succ(lane::ContactLocation::SUCCESSOR);
  lane::ContactLocation const pred(lane::ContactLocation::PREDECESSOR);
  lane::ContactTypeList defaultIntersectionContact;
  landmark::LandmarkId fakeTrafficLightId(landmark::LandmarkId::cMaxValue);

  auto laneId = toLaneId(lane.id);
  auto adlanePtr = mStore.getLanePtr(laneId);
  if (!adlanePtr) {
    return false;
  }
  auto adlane = *adlanePtr;

  switch (defaultIntersectionType) {
    case intersection::IntersectionType::HasWay:
      defaultIntersectionContact.push_back(lane::ContactType::FREE);
      break;
    case intersection::IntersectionType::Stop:
      defaultIntersectionContact.push_back(lane::ContactType::STOP);
      break;
    case intersection::IntersectionType::AllWayStop:
      defaultIntersectionContact.push_back(lane::ContactType::STOP_ALL);
      break;
    case intersection::IntersectionType::Yield:
      defaultIntersectionContact.push_back(lane::ContactType::YIELD);
      break;
    case intersection::IntersectionType::Crosswalk:
      defaultIntersectionContact.push_back(lane::ContactType::CROSSWALK);
      break;
    case intersection::IntersectionType::PriorityToRight:
      defaultIntersectionContact.push_back(lane::ContactType::PRIO_TO_RIGHT);
      break;
    case intersection::IntersectionType::PriorityToRightAndStraight:
      defaultIntersectionContact.push_back(
          lane::ContactType::PRIO_TO_RIGHT_AND_STRAIGHT);
      break;
    case intersection::IntersectionType::TrafficLight: {
      defaultIntersectionContact.push_back(lane::ContactType::TRAFFIC_LIGHT);
      // fake a traffic light to get the basic logic to work for now
      point::Geometry boundingBox;
      auto position = adlane.edgeLeft.ecefEdge[0];
      auto orientation = adlane.edgeLeft.ecefEdge[1];
      (void)addTrafficLight(access::PartitionId(0), fakeTrafficLightId,
                            defaultTrafficLightType, position, orientation,
                            boundingBox);
      break;
    }
    case intersection::IntersectionType::Unknown:
      // nothing to be done here
      break;
    default:
      access::getLogger()->error("Invalid defaultIntersectionType passed {}",
                                 static_cast<int>(defaultIntersectionType));
      return false;
  }

  if (lane.leftNeighbor != 0u) {
    auto restrictions = createRoadRestrictions();
    if (!add(laneId, toLaneId(lane.leftNeighbor), left, laneChange,
             restrictions)) {
      ok = false;
    }
  }
  if (lane.rightNeighbor != 0u) {
    auto restrictions = createRoadRestrictions();
    if (!add(laneId, toLaneId(lane.rightNeighbor), right, laneChange,
             restrictions)) {
      ok = false;
    }
  }

  for (auto const &successorId : lane.successors) {
    auto restrictions = createRoadRestrictions();
    if (!add(laneId, toLaneId(successorId), succ, laneContinuation,
             restrictions)) {
      ok = false;
    }

    if (adlane.type != lane::LaneType::INTERSECTION) {
      auto successorPtr = mStore.getLanePtr(toLaneId(successorId));
      if (!successorPtr) {
        return false;
      }
      auto successor = *successorPtr;
      if (successor.type == lane::LaneType::INTERSECTION) {
        //@todo: determine the correct right of way for that intersection
        if (!defaultIntersectionContact.empty()) {
          if (defaultIntersectionContact.front() ==
              lane::ContactType::TRAFFIC_LIGHT) {
            if (!add(laneId, toLaneId(successorId), succ,
                     defaultIntersectionContact, restrictions,
                     fakeTrafficLightId)) {
              ok = false;
            }
          } else {
            if (!add(laneId, toLaneId(successorId), succ,
                     defaultIntersectionContact, restrictions)) {
              ok = false;
            }
          }
        }
      }
    }
  }

  for (auto const &predecessorId : lane.predecessors) {
    auto restrictions = createRoadRestrictions();
    if (!add(laneId, toLaneId(predecessorId), pred, laneContinuation,
             restrictions)) {
      ok = false;
    }

    if (adlane.type != lane::LaneType::INTERSECTION) {
      auto predecessorPtr = mStore.getLanePtr(toLaneId(predecessorId));
      if (!predecessorPtr) {
        return false;
      }
      auto predecessor = *predecessorPtr;
      if (predecessor.type == lane::LaneType::INTERSECTION) {
        //@todo: determine the correct right of way for that intersection
        if (!defaultIntersectionContact.empty()) {
          if (defaultIntersectionContact.front() ==
              lane::ContactType::TRAFFIC_LIGHT) {
            if (!add(laneId, toLaneId(predecessorId), pred,
                     defaultIntersectionContact, restrictions,
                     fakeTrafficLightId)) {
              ok = false;
            }
          } else {
            if (!add(laneId, toLaneId(predecessorId), pred,
                     defaultIntersectionContact, restrictions)) {
              ok = false;
            }
          }
        }
      }
    }
  }

  for (auto const &overlappingLaneId : lane.overlaps) {
    if (!add(laneId, toLaneId(overlappingLaneId),
             lane::ContactLocation::OVERLAP, {lane::ContactType::UNKNOWN},
             restriction::Restrictions())) {
      ok = false;
    }
  }

  return ok;
}

bool AdMapFactory::convertToAdMap(
    ::opendrive::OpenDriveData &mapData,
    intersection::IntersectionType const defaultIntersectionType,
    landmark::TrafficLightType const defaultTrafficLightType) {
  bool ok = true;

  // @todo OpenDRIVE has not tags to define if right or left hand traffic
  // right now a function exists "toLaneDirection" inside DataTypeConversion.hpp
  // which calculates the lane direction given the lane attributes and a bool
  // value for right hand traffic
  set(access::TrafficType::RIGHT_HAND_TRAFFIC);

  // we first need to add all the lane ids and geometry to the store
  bool laneGenerationOk = (mapData.laneMap.size() != 0);
  for (auto const &element : mapData.laneMap) {
    if (!addLane(element.second)) {
      laneGenerationOk = false;
    }
  }
  ok = ok && laneGenerationOk;

  // add all the landmarks to the store
  bool landmarkGenerationOk = true;
  for (auto const &element : mapData.landmarks) {
    if (!addLandmark(element.second)) {
      landmarkGenerationOk = false;
    }
  }
  ok = ok && landmarkGenerationOk;

  // after all lanes are in the store we can add the contact information
  // including those derived from the landmarks
  bool contactGenerationOk = true;
  for (auto const &element : mapData.laneMap) {
    auto &lane = element.second;
    if (!addContactLanes(lane, defaultIntersectionType,
                         defaultTrafficLightType)) {
      contactGenerationOk = false;
    }
    if (!addSpecialContacts(lane, mapData.landmarks)) {
      contactGenerationOk = false;
    }
  }
  ok = ok && contactGenerationOk;

  if (!ok) {
    access::getLogger()->warn("AdMap conversion generated with errors");
    if (!laneGenerationOk) {
      access::getLogger()->warn("Lanes generated with errors");
    }
    if (!landmarkGenerationOk) {
      access::getLogger()->warn("Landmarks generated with errors");
    }
    if (!contactGenerationOk) {
      access::getLogger()->warn("Contacts generated with errors");
    }
  }

  // only errors during lane generation are considered to be a critical errors
  return laneGenerationOk;
}

}  // namespace opendrive
}  // namespace map
}  // namespace ad
