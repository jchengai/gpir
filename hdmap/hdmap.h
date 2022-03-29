/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <Eigen/Core>
#include <string>

#include "hdmap/routing/full_route.h"

namespace hdmap {

class HdMap {
 public:
  static HdMap& GetMap();

  bool LoadMap(const std::string& map_file, const std::string& pcd_file = "");

  bool CreateFullRoute(const Eigen::Vector2d& start, const Eigen::Vector2d& end,
                       const double start_heading, const double end_heading,
                       FullRoute* full_route);

  static LaneId NearestLane(const Eigen::Vector2d& position,
                            const double heading = 0.0,
                            const bool use_route_hint = true);

 private:
  HdMap() = default;
  HdMap(const HdMap&) = delete;
  HdMap& operator=(const HdMap&) = delete;
};
}  // namespace hdmap
