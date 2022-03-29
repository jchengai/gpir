/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <Eigen/Core>

#include "ad/map/route/FullRoute.hpp"
#include "hdmap/routing/full_route.h"

namespace hdmap {
class Routing {
 public:
  Routing() = default;
  ~Routing() = default;

  bool CreateFullRoute(const Eigen::Vector2d& start, const Eigen::Vector2d& end,
                       const double start_heading, const double end_heading,
                       FullRoute* full_route);

  inline bool has_route() const;
  inline const ad::map::route::FullRoute& ad_full_route() const;

 private:
  void DecideLaneSegmentBehavior(const LaneId end_lane_id,
                                 FullRoute* full_route);

  std::vector<LaneSegmentBehavior> DecideUnExtensibleLaneBehavior(
      const int index, std::vector<size_t> lanes);

  std::vector<LaneSegmentBehavior> DecideExtensibleLaneBehavior(
      const int index, std::vector<size_t> lanes);

  void PartitionLaneSegments(
      const ad::map::route::LaneSegmentList& lane_segments,
      const LaneId& end_land_id, std::vector<size_t>* extensible_lanes,
      std::vector<size_t>* inextensible_laness);

  size_t FindLaneInLaneSegments(
      const ad::map::route::LaneSegmentList& lane_segments, const LaneId& id);

 private:
  bool has_route_ = false;
  ::ad::map::route::FullRoute ad_full_route_;
};

// inline
inline bool Routing::has_route() const { return has_route_; }
inline const ad::map::route::FullRoute& Routing::ad_full_route() const {
  return ad_full_route_;
}

}  // namespace hdmap
