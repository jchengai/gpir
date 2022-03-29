/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "hdmap/hdmap.h"

#include "hdmap/hdmap_impl.h"

namespace hdmap {

static HdMapImpl hdmap_impl_;

HdMap& HdMap::GetMap() {
  static HdMap hdmap;
  return hdmap;
}

bool HdMap::LoadMap(const std::string& file, const std::string& pcd_file) {
  return hdmap_impl_.LoadMap(file, pcd_file);
}

bool HdMap::CreateFullRoute(const Eigen::Vector2d& start,
                            const Eigen::Vector2d& end,
                            const double start_heading,
                            const double end_heading, FullRoute* full_route) {
  return hdmap_impl_.CreateFullRoute(start, end, start_heading, end_heading,
                                     full_route);
}

LaneId HdMap::NearestLane(const Eigen::Vector2d& position, const double heading,
                          const bool use_route_hint) {
  if (use_route_hint) {
    return hdmap_impl_.NearestLane(position, heading);
  } else {
    return hdmap_impl_.NearestLaneNoHint(position, heading);
  }
}

}  // namespace hdmap
