/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include "common/base/state.h"
#include "planning_core/planning_common/obstacle.h"

namespace planning {

struct DataFrame {
  common::State state;
  std::vector<Obstacle> obstacles;
};

}  // namespace planning
