/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <Eigen/Core>

#include "ad/map/point/Operation.hpp"

namespace hdmap {

class AdMapConvertion {
 public:
  static ad::map::point::ENUPoint ToEnuPoint(const Eigen::Vector2d& position);

  static Eigen::Vector2d FromEnuPoint(const ad::map::point::ENUPoint& position);
};
}  // namespace hdmap
