/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "hdmap/routing/full_route.h"

namespace hdmap {

std::string ToString(const LaneSegmentBehavior& behavior) {
  if (behavior == LaneSegmentBehavior::kLeftChange)
    return "left";
  else if (behavior == LaneSegmentBehavior::kRightChange)
    return "right";
  else
    return "keep";
}
}  // namespace hdmap

std::ostream& operator<<(std::ostream& os,
                         const hdmap::LaneSegmentBehavior& behavior) {
  os << hdmap::ToString(behavior);
  return os;
}