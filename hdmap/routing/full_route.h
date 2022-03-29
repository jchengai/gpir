/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <unordered_map>

#include "hdmap/road_network/lane.h"

namespace hdmap {

// possible behavior ego may perform when in that lane
enum class LaneSegmentBehavior {
  kKeep = 0,
  kLeftChange = 1,
  kRightChange = 2,
  kNone = 3,
};

struct LaneSegment {
  LaneId id;
  double start_s = 0.0;
  double end_s = 0.0;
  double length = 0.0;
  double maximum_lane_keeping_length = 0.0;
  LaneId left_neighbor_id = 0;   // * zero means none
  LaneId right_neighbor_id = 0;  // * zero means none
  std::vector<LaneId> successors_id;
  std::vector<LaneId> presuccessor_id;
  std::vector<LaneSegmentBehavior> behavior;

  LaneSegment() = default;
};

typedef std::vector<LaneSegment> LaneSegments;

struct RoadSegment {
  int num_of_drivable_lanes = 0;
  std::vector<LaneSegment> lane_segments;
};

typedef std::vector<RoadSegment> RoadSegments;

struct FullRoute {
  Eigen::Vector2d start;
  Eigen::Vector2d end;
  LaneId target_lane_id;

  std::vector<RoadSegment> road_segments;
  std::unordered_map<LaneId, std::pair<int, int>, LaneIdHasher> lane_map;

  LaneSegment* GetLaneSegment(const LaneId& id) {
    auto index = lane_map.at(id);
    return &road_segments[index.first].lane_segments[index.second];
  }
};

std::string ToString(const LaneSegmentBehavior& behavior);
}  // namespace hdmap

// some helper functions
std::ostream& operator<<(std::ostream& os,
                         const hdmap::LaneSegmentBehavior& behavior);
