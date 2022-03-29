/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include "hdmap/routing/full_route.h"

namespace planning {

class RouteSegment {
 public:
  RouteSegment() = default;
  RouteSegment(const hdmap::LaneSegment& lane_segment)
      : id_(lane_segment.id), length_(lane_segment.length) {}

  inline hdmap::LaneId id() const { return id_; }
  inline hdmap::LaneId left_lane_id() const { return left_lane_id_; }
  inline hdmap::LaneId right_lane_id() const { return right_lane_id_; }
  inline hdmap::LaneId neighbor_id(const hdmap::LaneSegmentBehavior type) const;

  inline void set_id(const int id) { id_ = id; }
  inline void set_left_lane_id(const int id) { left_lane_id_ = id; }
  inline void set_right_lane_id(const int id) { right_lane_id_ = id; }

  inline hdmap::LaneSegmentBehavior main_action() const;
  inline const std::vector<hdmap::LaneSegmentBehavior>& alternative_actions()
      const;

  inline void set_main_action(hdmap::LaneSegmentBehavior action);
  inline void add_alternative_actions(hdmap::LaneSegmentBehavior action);

 private:
  hdmap::LaneId id_ = 0;
  hdmap::LaneId left_lane_id_ = 0;
  hdmap::LaneId right_lane_id_ = 0;
  double length_ = 0.0;

  hdmap::LaneSegmentBehavior main_action_ = hdmap::LaneSegmentBehavior::kNone;
  hdmap::LaneSegmentBehavior next_action_ = hdmap::LaneSegmentBehavior::kNone;
  std::vector<hdmap::LaneSegmentBehavior> alternative_actions_;
};

inline hdmap::LaneSegmentBehavior RouteSegment::main_action() const {
  return main_action_;
}

inline const std::vector<hdmap::LaneSegmentBehavior>&
RouteSegment::alternative_actions() const {
  return alternative_actions_;
}

inline void RouteSegment::set_main_action(hdmap::LaneSegmentBehavior action) {
  main_action_ = action;
}

inline void RouteSegment::add_alternative_actions(
    hdmap::LaneSegmentBehavior action) {
  alternative_actions_.emplace_back(action);
}

inline hdmap::LaneId RouteSegment::neighbor_id(
    const hdmap::LaneSegmentBehavior type) const {
  if (type == hdmap::LaneSegmentBehavior::kLeftChange)
    return left_lane_id_;
  else if (type == hdmap::LaneSegmentBehavior::kRightChange)
    return right_lane_id_;
  else
    return hdmap::LaneId(0);
}
}  // namespace planning
