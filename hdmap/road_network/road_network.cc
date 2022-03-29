/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "hdmap/road_network/road_network.h"

#include <iomanip>

std::ostream& operator<<(std::ostream& os, const hdmap::WayPoint& waypoint) {
  os << std::fixed << std::setprecision(4) << "[" << std::left
     << "x: " << std::setw(6) << waypoint.point.x() << ", "
     << "y: " << std::setw(6) << waypoint.point.y() << ", "
     << "s: " << std::setw(6) << waypoint.s << ", "
     << "0: " << std::setw(6) << waypoint.heading << "]\n";
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const std::vector<hdmap::WayPoint>& waypoints) {
  int count = 0;
  for (const auto& p : waypoints) os << count++ << ". " << p;
  return os;
}