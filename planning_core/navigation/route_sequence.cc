/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/navigation/route_sequence.h"

#include "hdmap/road_network/lane_map.h"

namespace planning {

void RouteSequence::Update(const Eigen::Vector2d& position) {
  if (empty()) return;
  constexpr double kEpsilon = 0.1;
  auto current_lane = hdmap::LaneMap::GetLane(at(current_index_).id());
  if (current_lane->GetArcLength(position) >
      current_lane->length() - kEpsilon) {
    if (current_index_ == size() - 1) {
      arrived_ = true;
    } else {
      if (current_index_ == size() - 2) {
        approaching_destination_ = true;
      }
      ++current_index_;
    }
  }
}

bool RouteSequence::IsWithInLane(const Eigen::Vector2d position) {
  auto current_lane = hdmap::LaneMap::GetLane(at(current_index_).id());
  auto proj = current_lane->GetProjection(position);
  if (std::fabs(proj.second) <= 0.3)
    return true;
  else
    return false;
}

void RouteSequence::RemoveOldestRoute() {
  pop_front();
  --current_index_;
}

void RouteSequence::ChangeMainAction(const hdmap::LaneSegmentBehavior type) {
  for (auto it = begin(); it < end(); ++it) {
    it->set_main_action(type);
  }
}

void RouteSequence::AddRoute(const RouteSegment& route_segment) {
  push_back(route_segment);
  arrived_ = false;
  approaching_destination_ = false;
}

void RouteSequence::Reset() {
  clear();
  current_index_ = 0;
  num_of_segments_ = 0;
  arrived_ = false;
  approaching_destination_ = false;
}
}  // namespace planning
