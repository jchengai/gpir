/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include "planning_core/navigation/navigation_map.h"

namespace planning {

class Planner {
 public:
  Planner() = default;
  virtual ~Planner() = default;

  virtual void Init() = 0;
  virtual void PlanOnce(NavigationMap* navigation_map_) = 0;
  virtual void LogDebugInfo() {}
};

}  // namespace planning
