/*
 * ----------------- BEGIN LICENSE BLOCK ---------------------------------
 *
 * Copyright (C) 2019-2020 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 *
 * ----------------- END LICENSE BLOCK -----------------------------------
 */

#include "opendrive/geometry/GeometryGenerator.hpp"

#include <algorithm>
#include <boost/math/tools/rational.hpp>
#include <iostream>

#include "opendrive/geometry/CenterLine.hpp"
#include "opendrive/geometry/LaneUtils.hpp"

#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H
#include <proj_api.h>

namespace opendrive {
namespace geometry {

double laneWidth(std::vector<LaneWidth> const &laneWidthVector, double s) {
  if (s < 0.) {
    return laneWidth(laneWidthVector, 0.);
  }

  for (auto it = laneWidthVector.rbegin(); it != laneWidthVector.rend(); it++) {
    if (s >= it->soffset) {
      auto poly = boost::array<double, 4>{{it->a, it->b, it->c, it->d}};
      return boost::math::tools::evaluate_polynomial(poly, s - it->soffset);
    }
  }
  return 0.0;
}

bool isInvalidLaneSection(::opendrive::LaneSection const &laneSection) {
  auto length = laneSection.end_position - laneSection.start_position;
  if (length < MinimumSegmentLength) {
    std::cerr << "Invalid lane section of length " << length << "\n";
    return true;
  }
  return false;
}

void calculateLaneSectionBounds(
    std::vector<::opendrive::LaneSection> &laneSections, double totalLength) {
  for (auto it = laneSections.begin(); it != laneSections.end(); it++) {
    if (it + 1 == laneSections.end()) {
      it->end_position = totalLength;
    } else {
      it->end_position = (it + 1)->start_position;
    }
  }
}

bool fixInvalidLaneSections(
    std::vector<::opendrive::LaneSection> &laneSections) {
  for (auto it = laneSections.begin(); it != laneSections.end(); it++) {
    // for the moment do nothing
    if (isInvalidLaneSection(*it)) {
      return false;
    }
  }
  return true;
}

void sortLanesByIndex(std::vector<::opendrive::LaneSection> &laneSections) {
  auto compare = [](::opendrive::LaneInfo const &infoA,
                    ::opendrive::LaneInfo const &infoB) {
    return abs(infoA.attributes.id) < abs(infoB.attributes.id);
  };

  for (auto it = laneSections.begin(); it != laneSections.end(); it++) {
    std::sort(it->left.begin(), it->left.end(), compare);
    std::sort(it->right.begin(), it->right.end(), compare);
  }
}

void generateTrafficSignal(TrafficSignalInformation &trafficSignalInfo,
                           CenterLine const &centerLine,
                           ::opendrive::LandmarkMap &landmarks) {
  Landmark landmark;
  auto directedPoint = centerLine.eval(trafficSignalInfo.start_position, false);
  directedPoint.ApplyLateralOffset(trafficSignalInfo.track_position);

  landmark.position = directedPoint.location;
  landmark.orientation = directedPoint.tangent + M_PI;

  if (trafficSignalInfo.orientation == "-") {
    landmark.orientation += M_PI;
  }

  landmark.id = trafficSignalInfo.id;
  try {
    landmark.type = stoi(trafficSignalInfo.type);
    landmark.subtype = stoi(trafficSignalInfo.subtype);
  } catch (...) {
    landmark.type = -1;
    landmark.subtype = -1;
  }

  if (landmark.id == -1) {
    landmark.id = landmarks.size() + 1;
    trafficSignalInfo.id = landmark.id;
  }
  landmarks[landmark.id] = landmark;
}

std::vector<LaneSection>::iterator getLaneSection(double s,
                                                  RoadInformation &roadInfo) {
  for (auto it = roadInfo.lanes.lane_sections.end() - 1;
       it >= roadInfo.lanes.lane_sections.begin(); --it) {
    if (s >= it->start_position) {
      return it;
    }
  }
  return roadInfo.lanes.lane_sections.end();
}

void addTrafficReferenceToLanes(
    TrafficSignalReference const &trafficSignalReference,
    RoadInformation &roadInfo, LaneMap &laneMap) {
  auto laneSectionIt =
      getLaneSection(trafficSignalReference.start_position, roadInfo);

  if (laneSectionIt == roadInfo.lanes.lane_sections.end()) {
    std::cerr
        << "addTrafficReferenceToLanes() traffic reference outside road\n";
    return;
  }

  auto laneSectionIndex =
      static_cast<uint32_t>(laneSectionIt -
                            roadInfo.lanes.lane_sections.begin()) +
      1;
  double s =
      parametricPosition(trafficSignalReference.start_position, *laneSectionIt);

  if (trafficSignalReference.orientation == "+") {
    for (auto &laneInfo : laneSectionIt->right) {
      SignalReference signalReference;
      signalReference.id = trafficSignalReference.id;
      signalReference.parametricPosition = s;
      signalReference.inLaneOrientation = true;
      auto id = laneId(roadInfo.attributes.id, laneSectionIndex,
                       laneInfo.attributes.id);
      laneMap[id].signalReferences.emplace_back(signalReference);
    }
  } else {
    for (auto &laneInfo : laneSectionIt->left) {
      SignalReference signalReference;
      signalReference.id = trafficSignalReference.id;
      signalReference.parametricPosition = s;
      signalReference.inLaneOrientation = false;
      auto id = laneId(roadInfo.attributes.id, laneSectionIndex,
                       laneInfo.attributes.id);
      laneMap[id].signalReferences.emplace_back(signalReference);
    }
  }
}

void addTrafficSignals(RoadInformation &roadInfo, CenterLine const &centerLine,
                       opendrive::OpenDriveData &mapData) {
  for (auto &trafficSignalInfo : roadInfo.traffic_signals) {
    generateTrafficSignal(trafficSignalInfo, centerLine, mapData.landmarks);

    // Create a signal reference to the given traffic signal
    TrafficSignalReference trafficSignalReference;
    trafficSignalReference.id = trafficSignalInfo.id;
    trafficSignalReference.orientation = trafficSignalInfo.orientation;
    trafficSignalReference.start_position = trafficSignalInfo.start_position;

    addTrafficReferenceToLanes(trafficSignalReference, roadInfo,
                               mapData.laneMap);
  }

  for (auto &trafficSignalReference : roadInfo.traffic_signal_references) {
    addTrafficReferenceToLanes(trafficSignalReference, roadInfo,
                               mapData.laneMap);
  }
}

bool addLane(OpenDriveData &mapData, RoadInformation const &roadInfo,
             LaneInfo const &laneInfo, Id laneId) {
  if (mapData.laneMap.count(laneId) > 0) {
    std::cerr << "Duplicated lane Id " << laneId << "\n";
    return false;
  }

  mapData.laneMap[laneId].leftEdge.clear();
  mapData.laneMap[laneId].rightEdge.clear();
  mapData.laneMap[laneId].index = laneInfo.attributes.id;
  mapData.laneMap[laneId].id = laneId;
  mapData.laneMap[laneId].type = laneInfo.attributes.type;
  mapData.laneMap[laneId].junction = roadInfo.attributes.junction;

  if (roadInfo.attributes.junction != -1) {
    mapData.intersectionLaneIds[roadInfo.attributes.junction].push_back(laneId);
  }

  return true;
}

bool initializeLaneMap(opendrive::OpenDriveData &mapData) {
  bool ok = true;
  for (auto const &roadInfo : mapData.roads) {
    auto const &roadId = roadInfo.attributes.id;
    auto const &laneSections = roadInfo.lanes.lane_sections;

    for (std::size_t k = 0; k != laneSections.size(); k++) {
      for (auto const &laneInfo : laneSections[k].left) {
        auto id = laneId(roadId, k + 1, laneInfo.attributes.id);
        if (!addLane(mapData, roadInfo, laneInfo, id)) {
          ok = false;
        }
      }

      for (auto const &laneInfo : laneSections[k].right) {
        auto id = laneId(roadId, k + 1, laneInfo.attributes.id);
        if (!addLane(mapData, roadInfo, laneInfo, id)) {
          ok = false;
        }
      }
    }
  }

  return ok;
}

void normalizeEdge(Id const id, std::string const &what, Edge &edge) {
  if (edge.size() <= 2u) {
    // if edge only consists of 2 points we could only remove the whole edge in
    // case the points are too near to each other
    return;
  }
  std::size_t pointsToDrop = 0u;
  Point previousEdgeDir(0., 0.);
  for (std::size_t i = 1u; i < edge.size(); ++i) {
    // i-pointsToDrop > 0
    if (pointsToDrop > 0u) {
      edge[i - pointsToDrop] = edge[i];
    }
    if (edge[i - pointsToDrop] == edge[i - pointsToDrop - 1]) {
      // drop identical points
      // std::cerr<< "normalizeEdge[" << id << "] dropping identical point from
      // " << what << " edge at index: " << i << std::endl;
      pointsToDrop++;
    } else {
      //  moving the middle line in a narrow curve often leads to artifacts
      //  (circles) which are removed by the below
      auto nextEdgeDir = edge[i - pointsToDrop] - edge[i - pointsToDrop - 1];
      if (previousEdgeDir != Point(0., 0.)) {
        auto dotProduct = previousEdgeDir.dot(nextEdgeDir);
        if (dotProduct < 0.) {
          // std::cerr<< "normalizeEdge[" << id << "] dropping extreme direction
          // changing point from " << what << " edge at index: " << i <<
          // std::endl;
          pointsToDrop++;
        } else {
          previousEdgeDir = nextEdgeDir;
        }
      } else {
        previousEdgeDir = nextEdgeDir;
      }
    }
  }
  if (pointsToDrop > 0u) {
    std::size_t const newEdgeSize =
        std::max(std::size_t(2u), edge.size() - pointsToDrop);
    // std::cerr<< "normalizeEdge[" << id << "] dropping points from " << what
    // << " edge: " << pointsToDrop << " remaining: " << newEdgeSize <<
    // std::endl;
    edge.resize(newEdgeSize, Point(0., 0.));
  }
}

bool generateRoadGeometry(RoadInformation &roadInfo,
                          opendrive::OpenDriveData &mapData) {
  bool ok = true;
  auto roadId = roadInfo.attributes.id;
  auto &laneSections = roadInfo.lanes.lane_sections;

  CenterLine centerLine;
  if (!generateCenterLine(roadInfo, centerLine)) {
    ok = false;
  }

  calculateLaneSectionBounds(laneSections, centerLine.length);
  if (!fixInvalidLaneSections(
          laneSections))  // currently just checks consistency
  {
    ok = false;
  }
  sortLanesByIndex(laneSections);

  addTrafficSignals(roadInfo, centerLine, mapData);

  auto samplingPoints = centerLine.samplingPoints();

  for (auto it = laneSections.begin(); it != laneSections.end(); it++) {
    auto &laneSection = *it;
    auto laneSectionIndex =
        static_cast<uint32_t>(it - laneSections.begin()) + 1;

    // the tolerance at the end is necessary so that the lane offset applies
    // only to the current lane section
    double s0 = laneSection.start_position;
    double s1 = (it + 1) == laneSections.end()
                    ? centerLine.length
                    : (it + 1)->start_position - 1e-12;

    if (s1 < s0) {
      std::cerr << "Invalid lane section length s1, s0\n";
      ok = false;
    }

    auto inRange = [&s0, &s1](double const &s) { return (s < s1) && (s > s0); };

    std::vector<double> localSamplingPoints;
    localSamplingPoints.push_back(s0);
    std::copy_if(samplingPoints.begin(), samplingPoints.end(),
                 std::back_inserter(localSamplingPoints), inRange);
    localSamplingPoints.push_back(s1);

    for (auto s : localSamplingPoints) {
      auto centerPoint = centerLine.eval(s);

      double borderOffset = 0.;
      for (auto &laneInfo : laneSection.left) {
        auto width = laneWidth(laneInfo.lane_width, s - s0);
        auto rightPoint = centerPoint;
        auto leftPoint = centerPoint;

        leftPoint.ApplyLateralOffset(borderOffset + width);
        rightPoint.ApplyLateralOffset(borderOffset);

        auto const id =
            laneId(roadId, laneSectionIndex, laneInfo.attributes.id);

        mapData.laneMap[id].leftEdge.push_back(leftPoint.location);
        mapData.laneMap[id].rightEdge.push_back(rightPoint.location);
        borderOffset += width;
      }

      // right lanes
      borderOffset = 0.;
      for (auto &laneInfo : laneSection.right) {
        auto width = laneWidth(laneInfo.lane_width, s - s0);
        auto rightPoint = centerPoint;
        auto leftPoint = centerPoint;

        leftPoint.ApplyLateralOffset(borderOffset);
        rightPoint.ApplyLateralOffset(borderOffset - width);

        auto const id =
            laneId(roadId, laneSectionIndex, laneInfo.attributes.id);

        mapData.laneMap[id].leftEdge.push_back(leftPoint.location);
        mapData.laneMap[id].rightEdge.push_back(rightPoint.location);
        borderOffset -= width;
      }
    }
  }

  for (auto &lane : mapData.laneMap) {
    normalizeEdge(lane.first, "left", lane.second.leftEdge);
    normalizeEdge(lane.first, "right", lane.second.rightEdge);
  }

  return ok;
}

double convertToMetersPerSecond(double const value, std::string const &units) {
  if (units == "m/s") {
    return value;
  } else if (units == "km/h") {
    return value / 3.6;
  } else if (units == "mph") {
    return value * 0.4470389;
  } else {
    std::cerr << "Unrecognized speed units\n";
    return 0.0;
  }
}

double speedAt(double s, std::vector<RoadSpeed> speed) {
  for (auto it = speed.rbegin(); it != speed.rend(); ++it) {
    if (s >= it->s) {
      return convertToMetersPerSecond(it->max, it->unit);
    }
  }

  std::cerr << "speedAt() Invalid parameter s\n";

  if (speed.size() > 0) {
    return convertToMetersPerSecond(speed.front().max, speed.front().unit);
  }

  return 0.;
}

// Creates a parametric speed entries vector using the given road speed as
// reference The range [s0, s1] is mapped to [0., 1.0]
std::vector<ParametricSpeed> parametricSpeed(double s0, double s1,
                                             std::vector<RoadSpeed> speed) {
  double length = s1 - s0;

  if (length <= 0) {
    std::cerr << "parametricSpeed() Invalid parameters: s1 <= s0\n";
    return {ParametricSpeed(speedAt(s0, speed))};
  }

  if (fabs(length) < MinimumSegmentLength) {
    std::cerr << "parametricSpeed() road segment too short length=" << length
              << "\n";
    return {ParametricSpeed(speedAt(s0, speed))};
  }

  std::vector<double> speedPoints;
  std::vector<ParametricSpeed> roadParametricSpeed;

  speedPoints.push_back(s0);
  for (auto it = speed.begin(); it != speed.end(); ++it) {
    if ((it->s > s0) && (it->s < s1)) {
      speedPoints.push_back(it->s);
    }
  }
  speedPoints.push_back(s1);

  for (auto it = speedPoints.begin(); it != speedPoints.end(); ++it) {
    ::opendrive::ParametricSpeed parametricSpeed;
    parametricSpeed.start = std::max(0., (*it - s0) / length);
    parametricSpeed.speed = speedAt(*it, speed);

    if ((it + 1 == speedPoints.end()) || (*(it + 1) > s1)) {
      parametricSpeed.end = 1.0;
    } else {
      parametricSpeed.end = (*(it + 1) - s0) / length;
    }
    roadParametricSpeed.push_back(parametricSpeed);
  }
  return roadParametricSpeed;
}

std::vector<ParametricSpeed> calculateLaneSpeed(LaneInfo const &laneInfo,
                                                double laneSectionLength) {
  if (laneSectionLength < MinimumSegmentLength) {
    std::cerr << "calculateLaneSpeed:: lane section length too short\n";
  }

  std::vector<ParametricSpeed> speed;
  for (auto it = laneInfo.lane_speed.begin(); it != laneInfo.lane_speed.end();
       ++it) {
    ::opendrive::ParametricSpeed parametricSpeed;
    parametricSpeed.start = it->soffset / laneSectionLength;
    parametricSpeed.speed = convertToMetersPerSecond(it->max_speed, it->unit);
    if (it + 1 == laneInfo.lane_speed.end()) {
      parametricSpeed.end = 1.0;
    } else {
      parametricSpeed.end = (it + 1)->soffset / laneSectionLength;
    }
    speed.push_back(parametricSpeed);
  }
  return speed;
}

void calculateSpeed(RoadInformation &roadInfo, LaneMap &laneMap) {
  auto &laneSections = roadInfo.lanes.lane_sections;

  auto &roadSpeed = roadInfo.attributes.speed;

  uint32_t laneSectionIndex = 1;
  for (auto &laneSection : laneSections) {
    std::vector<ParametricSpeed> laneSectionSpeed;
    if (roadSpeed.empty()) {
      // no speed info
      laneSectionSpeed.push_back(
          ParametricSpeed(convertToMetersPerSecond(50.0, "km/h")));
    } else {
      laneSectionSpeed = parametricSpeed(laneSection.start_position,
                                         laneSection.end_position, roadSpeed);
    }

    for (auto &laneInfo : laneSection.left) {
      auto id = laneId(roadInfo.attributes.id, laneSectionIndex,
                       laneInfo.attributes.id);
      if (laneInfo.lane_speed.empty()) {
        laneMap[id].speed = laneSectionSpeed;
      } else {
        laneMap[id].speed = calculateLaneSpeed(
            laneInfo, laneSection.end_position - laneSection.start_position);
      }
    }
    for (auto &laneInfo : laneSection.right) {
      auto id = laneId(roadInfo.attributes.id, laneSectionIndex,
                       laneInfo.attributes.id);
      if (laneInfo.lane_speed.empty()) {
        laneMap[id].speed = laneSectionSpeed;
      } else {
        laneMap[id].speed = calculateLaneSpeed(
            laneInfo, laneSection.end_position - laneSection.start_position);
      }
    }
    laneSectionIndex++;
  }
}

bool checkId(Id id, std::string const &text = "") {
  if (id < 1) {
    std::cerr << "checkId() invalid id " << id << " " << text << "\n";
    return false;
  }
  return true;
}

bool belongsToIntersection(::opendrive::RoadInformation const &roadInfo) {
  return roadInfo.attributes.junction == -1 ? false : true;
}

void setPredecessor(::opendrive::OpenDriveData &mapData,
                    ::opendrive::RoadInformation const &roadInfo,
                    ::opendrive::LaneInfo const &laneInfo) {
  auto const &contact = roadInfo.road_link.predecessor->contact_point;
  auto roadIt = findRoad(mapData, roadInfo.road_link.predecessor->id);

  // predecessor road is connected to the first lane section
  uint32_t thisLaneSectionIndex = 1;
  Id thisLaneId = laneId(roadInfo.attributes.id, thisLaneSectionIndex,
                         laneInfo.attributes.id);

  checkId(thisLaneId, "::SetPredecessor");

  if (roadIt != mapData.roads.end()) {
    auto const &roadId = roadIt->attributes.id;
    Id predecessorId{0u};
    if (contact == ::opendrive::ContactPoint::End) {
      predecessorId = laneId(roadId, roadIt->lanes.lane_sections.size(),
                             laneInfo.link->predecessor_id);
      checkAddSuccessor(mapData.laneMap[predecessorId],
                        mapData.laneMap[thisLaneId]);
    } else if (contact == ::opendrive::ContactPoint::Start) {
      predecessorId = laneId(roadId, 1, laneInfo.link->predecessor_id);
      checkAddPredecessor(mapData.laneMap[predecessorId],
                          mapData.laneMap[thisLaneId]);
    }
    checkAddPredecessor(mapData.laneMap[thisLaneId],
                        mapData.laneMap[predecessorId]);
  }
}

void setSuccessor(::opendrive::OpenDriveData &mapData,
                  ::opendrive::RoadInformation const &roadInfo,
                  ::opendrive::LaneInfo const &laneInfo) {
  auto const &contact = roadInfo.road_link.successor->contact_point;
  auto roadIt = findRoad(mapData, roadInfo.road_link.successor->id);

  // successor road is connected to the last lane section
  uint32_t thisLaneSectionIndex = roadInfo.lanes.lane_sections.size();
  Id thisLaneId = laneId(roadInfo.attributes.id, thisLaneSectionIndex,
                         laneInfo.attributes.id);

  checkId(thisLaneId, "::setSuccessor");

  if (roadIt != mapData.roads.end()) {
    auto const &roadId = roadIt->attributes.id;
    Id successorId{0u};
    if (contact == ::opendrive::ContactPoint::End) {
      successorId = laneId(roadId, roadIt->lanes.lane_sections.size(),
                           laneInfo.link->successor_id);
      checkAddSuccessor(mapData.laneMap[successorId],
                        mapData.laneMap[thisLaneId]);
    } else if (contact == ::opendrive::ContactPoint::Start) {
      successorId = laneId(roadId, 1, laneInfo.link->successor_id);
      checkAddPredecessor(mapData.laneMap[successorId],
                          mapData.laneMap[thisLaneId]);
    }
    checkAddSuccessor(mapData.laneMap[thisLaneId],
                      mapData.laneMap[successorId]);
  }
}

bool hasSuccessorRoad(RoadInformation const &roadInfo) {
  auto &link = roadInfo.road_link;
  return link.successor != nullptr &&
         link.successor->element_type == ::opendrive::ElementType::Road;
}

bool hasPredecessorRoad(RoadInformation const &roadInfo) {
  auto &link = roadInfo.road_link;
  return link.predecessor != nullptr &&
         link.predecessor->element_type == ::opendrive::ElementType::Road;
}

void setSuccessorPredecessor(::opendrive::OpenDriveData &mapData,
                             ::opendrive::RoadInformation const &roadInfo,
                             ::opendrive::LaneInfo const &laneInfo,
                             uint32_t const laneSectionIndex) {
  if (laneInfo.link == nullptr) {
    return;
  }

  Id currentLaneId =
      laneId(roadInfo.attributes.id, laneSectionIndex, laneInfo.attributes.id);
  auto &laneSections = roadInfo.lanes.lane_sections;

  // First handle predecessors
  if (laneSectionIndex == 1) {
    // set predecessor
    if (laneInfo.link->predecessor_id != 0 && hasPredecessorRoad(roadInfo)) {
      setPredecessor(mapData, roadInfo, laneInfo);
    }
  } else {
    if (laneInfo.link->predecessor_id != 0) {
      Id predecessorId = laneId(roadInfo.attributes.id, laneSectionIndex - 1,
                                laneInfo.link->predecessor_id);
      if (mapData.laneMap.find(predecessorId) != mapData.laneMap.end()) {
        checkAddPredecessor(mapData.laneMap[currentLaneId],
                            mapData.laneMap[predecessorId]);
      } else {
        std::cerr << "Warning: predecessorId for road "
                  << roadInfo.attributes.id << " lane "
                  << laneInfo.link->predecessor_id << " and section "
                  << laneSectionIndex - 1 << " does not exist" << std::endl;
      }
    }
  }

  // Then handle successors
  if (laneSectionIndex < laneSections.size()) {
    if (laneInfo.link->successor_id != 0) {
      Id successorId = laneId(roadInfo.attributes.id, laneSectionIndex + 1,
                              laneInfo.link->successor_id);
      checkAddSuccessor(mapData.laneMap[currentLaneId],
                        mapData.laneMap[successorId]);
    }
  } else {
    // set successor
    if (laneInfo.link->successor_id != 0 && hasSuccessorRoad(roadInfo)) {
      setSuccessor(mapData, roadInfo, laneInfo);
    }
  }
}

void setLeftRightNeighbor(::opendrive::OpenDriveData &mapData,
                          ::opendrive::RoadInformation const &roadInfo,
                          uint32_t const laneSectionIndex) {
  auto &laneSection = roadInfo.lanes.lane_sections[laneSectionIndex - 1];
  auto numLeftLanes = laneSection.left.size();
  auto numRightLanes = laneSection.right.size();

  for (std::size_t k = 0; k < laneSection.left.size(); k++) {
    auto &laneInfo = laneSection.left[k];
    Id thisLaneId = laneId(roadInfo.attributes.id, laneSectionIndex,
                           laneInfo.attributes.id);
    Id rightNeighborId{0u};
    Id leftNeighborId{0u};

    // right neighbor
    if (k == 0) {
      if (numRightLanes > 0) {
        rightNeighborId = laneId(roadInfo.attributes.id, laneSectionIndex,
                                 laneSection.right.front().attributes.id);
      }
    } else {
      rightNeighborId = laneId(roadInfo.attributes.id, laneSectionIndex,
                               laneSection.left[k - 1].attributes.id);
    }

    // left neighbor
    if (k + 1 < numLeftLanes) {
      leftNeighborId = laneId(roadInfo.attributes.id, laneSectionIndex,
                              laneSection.left[k + 1].attributes.id);
    }

    mapData.laneMap[thisLaneId].rightNeighbor = rightNeighborId;
    mapData.laneMap[thisLaneId].leftNeighbor = leftNeighborId;
  }
  for (std::size_t k = 0; k < laneSection.right.size(); k++) {
    auto &laneInfo = laneSection.right[k];
    Id thisLaneId = laneId(roadInfo.attributes.id, laneSectionIndex,
                           laneInfo.attributes.id);
    Id rightNeighborId{0u};
    Id leftNeighborId{0u};

    // left neighbor
    if (k == 0) {
      if (numLeftLanes > 0) {
        leftNeighborId = laneId(roadInfo.attributes.id, laneSectionIndex,
                                laneSection.left.front().attributes.id);
      }
    } else {
      leftNeighborId = laneId(roadInfo.attributes.id, laneSectionIndex,
                              laneSection.right[k - 1].attributes.id);
    }

    // right neighbor
    if (k + 1 < numRightLanes) {
      rightNeighborId = laneId(roadInfo.attributes.id, laneSectionIndex,
                               laneSection.right[k + 1].attributes.id);
    }

    mapData.laneMap[thisLaneId].rightNeighbor = rightNeighborId;
    mapData.laneMap[thisLaneId].leftNeighbor = leftNeighborId;
  }
}

void generateTopology(RoadInformation &roadInfo,
                      opendrive::OpenDriveData &mapData) {
  auto &laneSections = roadInfo.lanes.lane_sections;

  // left and right neighbors
  for (std::size_t i = 0; i < laneSections.size(); ++i) {
    setLeftRightNeighbor(mapData, roadInfo, i + 1);
  }
  for (std::size_t i = 0; i < laneSections.size(); ++i) {
    auto &laneSection = laneSections[i];
    auto laneSectionIndex = i + 1;

    for (std::size_t k = 0; k < laneSection.left.size(); k++) {
      auto &laneInfo = laneSection.left[k];
      setSuccessorPredecessor(mapData, roadInfo, laneInfo, laneSectionIndex);
    }
    for (std::size_t k = 0; k < laneSection.right.size(); k++) {
      auto &laneInfo = laneSection.right[k];
      setSuccessorPredecessor(mapData, roadInfo, laneInfo, laneSectionIndex);
    }
  }
}

void autoConnectIntersectionLanes(opendrive::OpenDriveData &mapData,
                                  double const overlapMargin) {
  auto &laneMap = mapData.laneMap;
  auto &intersectionLaneIds = mapData.intersectionLaneIds;

  for (auto &element : intersectionLaneIds) {
    auto &lanes = element.second;
    for (auto leftIt = lanes.begin(); leftIt != lanes.end(); leftIt++) {
      for (auto rightIt = leftIt; rightIt != lanes.end(); rightIt++) {
        if (lanesOverlap(laneMap[*leftIt], laneMap[*rightIt], overlapMargin)) {
          laneMap[*leftIt].overlaps.push_back(*rightIt);
          laneMap[*rightIt].overlaps.push_back(*leftIt);
        }
        switch (contactPlace(laneMap[*leftIt], laneMap[*rightIt])) {
          case ContactPlace::LeftLeft:
            // lane and neighbors need inversion
            invertLaneAndNeighbors(laneMap, mapData.laneMap[*rightIt]);
          // fallthrough
          case ContactPlace::LeftRight:
            laneMap[*leftIt].leftNeighbor = *rightIt;
            laneMap[*rightIt].rightNeighbor = *leftIt;
            break;
          case ContactPlace::RightRight:
            // lane and neighbors need inversion
            invertLaneAndNeighbors(laneMap, mapData.laneMap[*rightIt]);
          // fallthrough
          case ContactPlace::RightLeft:
            laneMap[*leftIt].rightNeighbor = *rightIt;
            laneMap[*rightIt].leftNeighbor = *leftIt;
            break;
          case ContactPlace::Overlap:
          case ContactPlace::None:
            // nothing to be done
            break;
        }
      }
    }
  }
}

double length(::opendrive::Edge const &edge) {
  double edgeLength = 0.0;
  for (std::size_t i = 1u; i < edge.size(); ++i) {
    edgeLength += (edge[i - 1] - edge[i]).norm();
  }
  return edgeLength;
}

bool checkLaneConsistency(opendrive::OpenDriveData &mapData) {
  bool ok = true;
  auto &laneMap = mapData.laneMap;
  std::vector<Id> deletionLanes;
  for (auto element : laneMap) {
    auto &lane = element.second;
    double leftLength = length(lane.leftEdge);
    double rightLength = length(lane.rightEdge);

    if (leftLength < 2.0e-10 || rightLength < 2.0e-10) {
      std::cerr << "checkLaneConsistency:: Invalid lane geometry for lane "
                << lane.id << "\n";
      deletionLanes.push_back(lane.id);
      ok = false;
    }
  }

  return ok;
}

bool convertToGeoPoints(opendrive::OpenDriveData &mapData) {
  auto projPtr = pj_init_plus(mapData.geoReference.projection.c_str());
  if (projPtr == nullptr) {
    if (std::isnan(mapData.geoReference.longitude) ||
        std::isnan(mapData.geoReference.latitude)) {
      return false;
    }

    std::string defaultProjection =
        "+proj=tmerc +ellps=WGS84 +lon_0=" +
        std::to_string(mapData.geoReference.longitude) +
        " +lat_0=" + std::to_string(mapData.geoReference.latitude);
    projPtr = pj_init_plus(defaultProjection.c_str());
    std::cerr << "Using default projection: " << defaultProjection << "\n";
    if (projPtr == nullptr) {
      std::cerr << "Unknown error while creating the projection\n";
      return false;
    }
  }

  auto convertENUToGeo = [&projPtr](Point &point) {
    projXY enuPoint;
    enuPoint.u = point.x;
    enuPoint.v = point.y;

    auto geoPoint = pj_inv(enuPoint, projPtr);
    point.x = geoPoint.u * RAD_TO_DEG;
    point.y = geoPoint.v * RAD_TO_DEG;
  };

  for (auto &element : mapData.laneMap) {
    auto &lane = element.second;
    for (auto &point : lane.leftEdge) {
      convertENUToGeo(point);
    }
    for (auto &point : lane.rightEdge) {
      convertENUToGeo(point);
    }
  }

  for (auto &element : mapData.landmarks) {
    convertENUToGeo(element.second.position);
  }

  pj_dalloc(projPtr);
  return true;
}

bool GenerateGeometry(opendrive::OpenDriveData &open_drive_data,
                      double const overlapMargin) {
  bool ok = initializeLaneMap(open_drive_data);

  for (auto &roadInfo : open_drive_data.roads) {
    if (!generateRoadGeometry(roadInfo, open_drive_data)) {
      ok = false;
    }
  }

  for (auto &roadInfo : open_drive_data.roads) {
    calculateSpeed(roadInfo, open_drive_data.laneMap);
  }
  for (auto &roadInfo : open_drive_data.roads) {
    generateTopology(roadInfo, open_drive_data);
  }

  if (!checkLaneConsistency(open_drive_data)) {
    ok = false;
  }
  autoConnectIntersectionLanes(open_drive_data, overlapMargin);

  if (!convertToGeoPoints(open_drive_data)) {
    ok = false;
  }

  return ok;
}
}  // namespace geometry
}  // namespace opendrive
