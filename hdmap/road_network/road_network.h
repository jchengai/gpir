/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <Eigen/Core>
#include <iostream>
#include <utility>

#include "ad/map/lane/Lane.hpp"

namespace hdmap {

using LaneId = ::ad::map::lane::LaneId;

enum class LaneTurnningType : size_t {
  kNoTurn = 1,
  kLeftTurn = 2,
  kRightTurn = 3
};

enum class LaneNeighborType : size_t {
  kLeft = 0,
  kRight = 1,
  kUnknown = 2,
};

enum class LaneConnectionType : size_t {
  kNext = 0,
  kPrevious = 1,
  kUnknown = 2,
};

struct WayPoint {
  double s = 0.0;
  double heading = 0.0;
  Eigen::Vector2d point;
};

struct NeighborLane {
  LaneId id;
  LaneNeighborType type = LaneNeighborType::kUnknown;
  bool lane_changable = false;
  ::ad::map::lane::Lane::ConstPtr admap_lane;
};

struct ConnectedLane {
  LaneId id;
  LaneConnectionType type = LaneConnectionType::kUnknown;
};

struct LaneIdHasher {
  std::size_t operator()(const LaneId& id) const {
    return static_cast<std::size_t>(id);
  }
};

using SLPair = std::pair<double, double>;

}  // namespace hdmap

// helper functions
std::ostream& operator<<(std::ostream& os, const hdmap::WayPoint& waypoints);
std::ostream& operator<<(std::ostream& os,
                         const std::vector<hdmap::WayPoint>& waypoints);
