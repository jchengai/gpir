/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "planning_core/planning_common/obstacle.h"

namespace planning {

void Obstacle::SetBoundingBox(const double length, const double width,
                              const double height) {
  height_ = height;
  bbox_ = common::Box2D(state_.position, length, width, state_.heading, height);
}

}  // namespace planning
