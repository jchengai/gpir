/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <deque>

#include "hdmap/routing/full_route.h"
#include "planning_core/navigation/route_segment.h"

namespace planning {

class RouteSequence : public std::deque<RouteSegment> {
 public:
  RouteSequence() = default;

  void Update(const Eigen::Vector2d& position);
  void RemoveOldestRoute();
  void AddRoute(const RouteSegment& route_segment);
  void ChangeMainAction(const hdmap::LaneSegmentBehavior type);
  bool IsWithInLane(const Eigen::Vector2d position);

  inline const RouteSegment& current_route() { return at(current_index_); }
  inline const RouteSegment& next_route() { return at(current_index_ + 1); }
  inline int current_index() const { return current_index_; }
  inline hdmap::LaneSegmentBehavior main_action() const;
  inline bool ApproachingDestination() { return approaching_destination_; }
  inline bool arrived() const { return arrived_; }
  inline bool approaching() const { return approaching_destination_; }
  void Reset();

 private:
  int num_of_segments_ = 0;
  int current_index_ = 0;
  bool arrived_ = false;
  bool approaching_destination_ = false;
};

// inline
inline hdmap::LaneSegmentBehavior RouteSequence::main_action() const {
  return at(current_index_).main_action();
}
}  // namespace planning
