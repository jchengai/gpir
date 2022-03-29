// ----------------- BEGIN LICENSE BLOCK ---------------------------------
//
// Copyright (C) 2018-2019 Intel Corporation
//
// SPDX-License-Identifier: MIT
//
// ----------------- END LICENSE BLOCK -----------------------------------

#include "ad/map/access/Factory.hpp"

#include <algorithm>

#include "../lane/LaneOperationPrivate.hpp"
#include "ad/map/access/Logging.hpp"
#include "ad/map/access/Store.hpp"
#include "ad/map/landmark/LandmarkOperation.hpp"
#include "ad/map/lane/LaneOperation.hpp"
#include "ad/map/point/GeometryOperation.hpp"
#include "ad/map/point/Transform.hpp"
#include "ad/map/restriction/ParametricRangeAttributeOperation.hpp"

namespace ad {
namespace map {
namespace access {

bool Factory::add(PartitionId part_id, const lane::LaneId &id,
                  lane::LaneType type, lane::LaneDirection dir) {
  auto insertResult = mStore.lane_map_.insert({id, lane::Lane::Ptr()});
  auto &lanePtr = insertResult.first->second;
  if (insertResult.second) {
    lanePtr = std::make_shared<lane::Lane>();
    lanePtr->id = id;
    mStore.part_lane_map_[part_id].push_back(id);
  }
  lanePtr->type = type;
  lanePtr->direction = dir;
  return insertResult.second;
}

lane::LaneId Factory::add(PartitionId pid, const point::GeoEdge &left_geo,
                          const point::GeoEdge &right_geo) {
  lane::LaneId new_lane_id = getNextLaneId();
  if (add(pid, new_lane_id, lane::LaneType::NORMAL,
          lane::LaneDirection::POSITIVE)) {
    point::CoordinateTransform cf;
    point::ECEFEdge left_ecef;
    point::ECEFEdge right_ecef;
    cf.convert(left_geo, left_ecef);
    cf.convert(right_geo, right_ecef);
    point::Geometry const left_edge = point::createGeometry(left_ecef, false);
    point::Geometry const right_edge = point::createGeometry(right_ecef, false);
    set(new_lane_id, left_edge, right_edge);
    return new_lane_id;
  } else {
    return lane::LaneId();
  }
}

lane::LaneId Factory::add(PartitionId pid, const point::ECEFEdge &left_ecef,
                          const point::ECEFEdge &right_ecef,
                          const lane::LaneId &lane_id_0,
                          const lane::LaneId &lane_id_1) {
  lane::LaneId new_lane_id = getNextLaneId();
  if (add(pid, new_lane_id, lane::LaneType::INTERSECTION,
          lane::LaneDirection::POSITIVE)) {
    point::Geometry const left_edge = point::createGeometry(left_ecef, false);
    point::Geometry const right_edge = point::createGeometry(right_ecef, false);
    set(new_lane_id, left_edge, right_edge);
    if (!autoConnect(new_lane_id, lane_id_0) ||
        !autoConnect(new_lane_id, lane_id_1) ||
        !autoConnect(lane_id_0, new_lane_id) ||
        !autoConnect(lane_id_1, new_lane_id)) {
      throw std::runtime_error("AutoConnect failed");
    }
    return new_lane_id;
  } else {
    return lane::LaneId();
  }
}

bool Factory::addLandmark(PartitionId part_id, const landmark::LandmarkId &id,
                          const landmark::LandmarkType type,
                          const point::ECEFPoint &position,
                          const point::ECEFPoint &orientation,
                          const point::Geometry &bounding_box) {
  return add(part_id, id, type, position, orientation, bounding_box,
             landmark::TrafficLightType::INVALID,
             landmark::TrafficSignType::INVALID, "none");
}

bool Factory::addTrafficLight(PartitionId part_id,
                              const landmark::LandmarkId &id,
                              const landmark::TrafficLightType type,
                              const point::ECEFPoint &position,
                              const point::ECEFPoint &orientation,
                              const point::Geometry &bounding_box) {
  return add(part_id, id, landmark::LandmarkType::TRAFFIC_LIGHT, position,
             orientation, bounding_box, type,
             landmark::TrafficSignType::INVALID, "none");
}

bool Factory::addTrafficSign(PartitionId part_id,
                             const landmark::LandmarkId &id,
                             const landmark::TrafficSignType type,
                             const point::ECEFPoint &position,
                             const point::ECEFPoint &orientation,
                             const point::Geometry &bounding_box,
                             const std::string &text) {
  return add(part_id, id, landmark::LandmarkType::TRAFFIC_SIGN, position,
             orientation, bounding_box, landmark::TrafficLightType::INVALID,
             type, text);
}

bool Factory::add(PartitionId part_id, const landmark::LandmarkId &id,
                  const landmark::LandmarkType type,
                  const point::ECEFPoint &position,
                  const point::ECEFPoint &orientation,
                  const point::Geometry &bounding_box,
                  const landmark::TrafficLightType traffic_light_type,
                  const landmark::TrafficSignType traffic_sign_type,
                  const std::string &text) {
  auto insertResult =
      mStore.landmark_map_.insert({id, landmark::Landmark::Ptr()});
  auto &landmarkPtr = insertResult.first->second;
  if (insertResult.second) {
    landmarkPtr = std::make_shared<landmark::Landmark>();
    landmarkPtr->id = id;
    mStore.part_landmark_map_[part_id].push_back(id);
  }
  landmarkPtr->type = type;
  landmarkPtr->position = position;
  landmarkPtr->orientation = orientation;
  landmarkPtr->boundingBox = bounding_box;
  landmarkPtr->trafficLightType = traffic_light_type;
  landmarkPtr->trafficSignType = traffic_sign_type;
  landmarkPtr->supplementaryText = text;
  // @todo: Do we need to ensure that we handle only valid objects?
  return insertResult.second;
}

//////////////////////////////////////////
// Atomic Operations -- Add Lane Attribute

bool Factory::add(const lane::LaneId &id,
                  const lane::ContactLane &contact_lane) {
  auto it = mStore.lane_map_.find(id);
  if (it != mStore.lane_map_.end()) {
    lane::Lane::Ptr lane = it->second;
    if (lane) {
      for (auto const &existing_contact_lane : lane->contactLanes) {
        if (contact_lane == existing_contact_lane) {
          return true;
        }
      }
      lane->contactLanes.push_back(contact_lane);
      return true;
    }
  }
  access::getLogger()->error("Cannot add contact lane of lane. {}", id);
  return false;
}

bool Factory::add(const lane::LaneId &id,
                  const lane::ContactLaneList &contact_lanes) {
  for (auto contact_lane : contact_lanes) {
    if (!add(id, contact_lane)) {
      access::getLogger()->error("Cannot add contact lanes of lane. {}", id);
      return false;
    }
  }
  return true;
}

bool Factory::add(const lane::LaneId &id,
                  const restriction::SpeedLimit &speedLimit) {
  auto it = mStore.lane_map_.find(id);
  if (it != mStore.lane_map_.end()) {
    lane::Lane::Ptr lane = it->second;
    if (lane) {
      if (restriction::doesRangeAttributeOverlap(lane->speedLimits,
                                                 speedLimit)) {
        access::getLogger()->warn(
            "Lane para-speed overlaps existing value!? {}, {}", id, speedLimit);
      }
      restriction::insertRangeAttribute(lane->speedLimits, speedLimit);
      return true;
    }
  }
  access::getLogger()->error("Cannot add restrictions of lane. {}", id);
  return false;
}

bool Factory::add(const lane::LaneId &id,
                  const landmark::LandmarkId &landmarkId) {
  if (!isValid(landmarkId)) {
    access::getLogger()->error("Cannot add landmark with invalid id. {}",
                               landmarkId);
    return false;
  }
  auto it = mStore.lane_map_.find(id);
  if (it != mStore.lane_map_.end()) {
    lane::Lane::Ptr lane = it->second;
    if (lane) {
      for (auto existing_visible_landmark : lane->visibleLandmarks) {
        if (landmarkId == existing_visible_landmark) {
          return true;
        }
      }
      lane->visibleLandmarks.push_back(landmarkId);
      return true;
    }
  }
  access::getLogger()->error("Cannot add landmark of lane. {}", id);
  return false;
}

bool Factory::add(const lane::LaneId &id,
                  const restriction::Restriction &restriction, bool andx) {
  auto it = mStore.lane_map_.find(id);
  if (it != mStore.lane_map_.end()) {
    lane::Lane::Ptr lane = it->second;
    if (lane) {
      if (andx) {
        lane->restrictions.conjunctions.push_back(restriction);
      } else {
        lane->restrictions.disjunctions.push_back(restriction);
      }
      return true;
    }
  }
  access::getLogger()->error("Cannot add restrictions of lane. {}", id);
  return false;
}

bool Factory::add(const lane::LaneId &id_from, const lane::LaneId &id_to,
                  const lane::ContactLocation location,
                  const lane::ContactTypeList &types,
                  const restriction::Restrictions &restrs) {
  if (std::find(types.begin(), types.end(), lane::ContactType::TRAFFIC_LIGHT) !=
      types.end()) {
    access::getLogger()->error(
        "Trying to add traffic light contact without traffic light id. {}",
        id_to);
    return false;
  }

  lane::ContactLane contactLane;
  contactLane.toLane = id_to;
  contactLane.location = location;
  contactLane.types = types;
  contactLane.restrictions = restrs;
  contactLane.trafficLightId = landmark::LandmarkId();
  return add(id_from, contactLane);
}

bool Factory::add(const lane::LaneId &id_from, const lane::LaneId &id_to,
                  const lane::ContactLocation location,
                  const lane::ContactTypeList &types,
                  const restriction::Restrictions &restrs,
                  const landmark::LandmarkId &traffic_light) {
  if (!isValid(traffic_light)) {
    access::getLogger()->error(
        "Cannot add contact with invalid traffic light id. {}", traffic_light);
    return false;
  }

  if (std::find(types.begin(), types.end(), lane::ContactType::TRAFFIC_LIGHT) ==
      types.end()) {
    access::getLogger()->error(
        "Cannot add traffic light contact due to wrong type. {}",
        traffic_light);
    return false;
  }

  lane::ContactLane contactLane;
  contactLane.toLane = id_to;
  contactLane.location = location;
  contactLane.types = types;
  contactLane.restrictions = restrs;
  contactLane.trafficLightId = traffic_light;
  return add(id_from, contactLane);
}

//////////////////////////////////////////
// Atomic Operations -- Set Lane Attribute

bool Factory::set(const lane::LaneId &id, lane::LaneDirection direction) {
  auto it = mStore.lane_map_.find(id);
  if (it != mStore.lane_map_.end()) {
    lane::Lane::Ptr lane = it->second;
    if (lane) {
      lane->direction = direction;
      return true;
    }
  }
  access::getLogger()->error("Cannot set direction of lane. {}", id);
  return false;
}

bool Factory::set(const lane::LaneId &id, lane::LaneType type) {
  auto it = mStore.lane_map_.find(id);
  if (it != mStore.lane_map_.end()) {
    lane::Lane::Ptr lane = it->second;
    if (lane) {
      lane->type = type;
      return true;
    }
  }
  access::getLogger()->error("Cannot set type of lane. {}", id);
  return false;
}

bool Factory::set(const lane::LaneId &id,
                  lane::ComplianceVersion compliance_ver) {
  auto it = mStore.lane_map_.find(id);
  if (it != mStore.lane_map_.end()) {
    lane::Lane::Ptr lane = it->second;
    if (lane) {
      lane->complianceVersion = compliance_ver;
      return true;
    }
  }
  access::getLogger()->error("Cannot set compliance of lane. {}", id);
  return false;
}

bool Factory::set(const lane::LaneId &id, const point::Geometry &edge_left,
                  const point::Geometry &edge_right) {
  auto it = mStore.lane_map_.find(id);
  if (it != mStore.lane_map_.end()) {
    lane::Lane::Ptr lane = it->second;
    if (lane) {
      lane->edgeLeft = edge_left;
      lane->edgeRight = edge_right;
      lane->boundingSphere = point::calcBoundingSphere(edge_left, edge_right);
      lane::updateLaneLengths(*lane);
      return true;
    }
  }
  access::getLogger()->error("Cannot set edges of lane. {}", id);
  return false;
}

bool Factory::set(const lane::LaneId &id, const physics::Speed &maxSpeed) {
  auto it = mStore.lane_map_.find(id);
  if (it != mStore.lane_map_.end()) {
    lane::Lane::Ptr lane = it->second;
    if (lane) {
      lane->speedLimits.clear();
      restriction::SpeedLimit speedLimit;
      speedLimit.lanePiece.minimum = physics::ParametricValue(0.);
      speedLimit.lanePiece.maximum = physics::ParametricValue(1.);
      speedLimit.speedLimit = maxSpeed;
      lane->speedLimits.push_back(speedLimit);
      return true;
    }
  }
  access::getLogger()->error("Cannot set speed limit for the. {}", id);
  return false;
}

bool Factory::set(const lane::LaneId &id,
                  const restriction::Restrictions &restrictions) {
  auto it = mStore.lane_map_.find(id);
  if (it != mStore.lane_map_.end()) {
    lane::Lane::Ptr lane = it->second;
    if (lane) {
      lane->restrictions = restrictions;
      return true;
    }
  }
  access::getLogger()->error("Cannot set restrictions of lane. {}", id);
  return false;
}

bool Factory::set(const lane::LaneId &id,
                  const landmark::LandmarkIdList &landmarks) {
  auto it = mStore.lane_map_.find(id);
  if (it != mStore.lane_map_.end()) {
    lane::Lane::Ptr lane = it->second;
    if (lane) {
      lane->visibleLandmarks = landmarks;
      return true;
    }
  }
  access::getLogger()->error("Cannot set bounding sphere of lane. {}", id);
  return false;
}

//////////////////////////////////////////
// Atomic Operations -- Set Map Meta Data

bool Factory::set(const TrafficType &traffic_type) {
  if (traffic_type != TrafficType::INVALID) {
    mStore.meta_data_.trafficType = traffic_type;
    return true;
  }

  return false;
}

/////////////////////////////////////
// Atomic operations -- remove things

bool Factory::deleteLane(lane::LaneId id) {
  if (lane::isValid(id)) {
    auto const erased = mStore.lane_map_.erase(id);
    if (erased > 0u) {
      bool removed_from_part_map = false;
      for (auto part_id_and_lane_ids : mStore.part_lane_map_) {
        auto const removeIter =
            std::remove(part_id_and_lane_ids.second.begin(),
                        part_id_and_lane_ids.second.end(), id);
        if (removeIter != part_id_and_lane_ids.second.end()) {
          part_id_and_lane_ids.second.erase(removeIter,
                                            part_id_and_lane_ids.second.end());
          removed_from_part_map = true;
          break;
        }
      }
      if (!removed_from_part_map) {
        access::getLogger()->error("Lane not in the Partitions?. {}", id);
      }
      return true;
    } else {
      access::getLogger()->warn("Lane is not in the Store. {}", id);
    }
  } else {
    access::getLogger()->error(
        "Invalid lane identifier passed to Factory::deleteLane()");
  }
  return false;
}

bool Factory::deleteContacts(lane::LaneId id, lane::LaneId to_id) {
  if (lane::isValid(id) && lane::isValid(to_id)) {
    auto it = mStore.lane_map_.find(id);
    if (it != mStore.lane_map_.end()) {
      lane::Lane::Ptr lane = it->second;
      if (lane) {
        lane::ContactLaneList &cls = lane->contactLanes;
        cls.erase(std::remove_if(cls.begin(), cls.end(),
                                 [to_id](const lane::ContactLane &cl) {
                                   return cl.toLane == to_id;
                                 }));
        return true;
      }
    }
  } else {
    access::getLogger()->error(
        "Invalid lane identifier passed to Factory::deleteContacts()");
  }
  return false;
}

bool Factory::deleteLandmark(landmark::LandmarkId id) {
  if (landmark::isValid(id)) {
    auto const erased = mStore.landmark_map_.erase(id);
    if (erased > 0u) {
      bool removed_from_part_map = false;
      for (auto part_id_and_landmark_ids : mStore.part_landmark_map_) {
        auto const removeIter =
            std::remove(part_id_and_landmark_ids.second.begin(),
                        part_id_and_landmark_ids.second.end(), id);
        if (removeIter != part_id_and_landmark_ids.second.end()) {
          part_id_and_landmark_ids.second.erase(
              removeIter, part_id_and_landmark_ids.second.end());
          removed_from_part_map = true;
          break;
        }
      }
      if (!removed_from_part_map) {
        access::getLogger()->error("Landmark not in a partition. {}", id);
      }
      return true;
    } else {
      access::getLogger()->warn("Landmark is not in the Store. {}", id);
    }
  } else {
    access::getLogger()->error(
        "Invalid landmark identifier passed to Factory::deleteLandmark()");
  }
  return false;
}

////////////////
// Other Methods

bool Factory::autoConnect(lane::LaneId from_id, lane::LaneId to_id) {
  bool ok = false;
  if (lane::isValid(from_id) && lane::isValid(to_id)) {
    lane::Lane::ConstPtr from_lane = mStore.getLanePtr(from_id);
    if (from_lane) {
      lane::Lane::ConstPtr to_lane = mStore.getLanePtr(to_id);
      if (to_lane) {
        if (lane::getContactLocation(*from_lane, to_id) ==
            lane::ContactLocation::INVALID) {
          static constexpr auto pred = lane::ContactLocation::PREDECESSOR;
          static constexpr auto succ = lane::ContactLocation::SUCCESSOR;
          lane::ContactTypeList types = {lane::ContactType::FREE};
          restriction::Restrictions restrs;
          if (lane::getStartPoint(*from_lane) ==
              lane::getStartPoint(*to_lane)) {
            ok = add(from_id, to_id, pred, types, restrs);
          } else if (lane::getStartPoint(*from_lane) ==
                     lane::getEndPoint(*to_lane)) {
            ok = add(from_id, to_id, pred, types, restrs);
          } else if (lane::getEndPoint(*from_lane) ==
                     lane::getStartPoint(*to_lane)) {
            ok = add(from_id, to_id, succ, types, restrs);
          } else if (lane::getEndPoint(*from_lane) ==
                     lane::getEndPoint(*to_lane)) {
            ok = add(from_id, to_id, succ, types, restrs);
          }
        }
      } else {
        access::getLogger()->error(
            "Non-existing to-lane Factory::autoConnect()");
      }
    } else {
      access::getLogger()->error(
          "Non-existing from-lane Factory::autoConnect()");
    }
  } else {
    access::getLogger()->error(
        "Invalid lane identifier passed to Factory::autoConnect()");
  }
  return ok;
}

/////////////
// Aux Methods

lane::LaneId Factory::getNextLaneId() const {
  lane::LaneId new_lane_id(0);
  for (auto it : mStore.lane_map_) {
    new_lane_id = std::max(new_lane_id, it.second->id);
  }
  new_lane_id += lane::LaneId(1);
  return new_lane_id;
}

}  // namespace access
}  // namespace map
}  // namespace ad
