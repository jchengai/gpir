/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "hdmap/road_network/lane_map.h"

namespace hdmap {

LaneMapType LaneMap::lane_map_;

LaneMapType* LaneMap::mutable_lane_map() { return &lane_map_; }

std::shared_ptr<Lane> LaneMap::GetLane(const LaneId& id) {
  if (lane_map_.find(id) != lane_map_.end())
    return lane_map_[id];
  else
    return nullptr;
}
}  // namespace hdmap
