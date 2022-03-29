/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "hdmap/utils/admap_convertion.h"

namespace hdmap {

ad::map::point::ENUPoint AdMapConvertion::ToEnuPoint(
    const Eigen::Vector2d& position) {
  return ad::map::point::createENUPoint(position.x(), position.y(), 0.0);
}

Eigen::Vector2d AdMapConvertion::FromEnuPoint(
    const ad::map::point::ENUPoint& position) {
  return Eigen::Vector2d(position.x, position.y);
}
}  // namespace hdmap
