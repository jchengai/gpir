/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/st_plan/st_node.h"

#include <cmath>
#include <limits>

namespace planning {

double StNode::ref_v_ = 0.0;
StNodeWeights StNode::weights_;

std::unique_ptr<StNode> StNode::Forward(const double delta_t,
                                        const double a) const {
  std::unique_ptr<StNode> st_node = std::make_unique<StNode>();
  st_node->a = a;
  st_node->t = t + delta_t;
  st_node->s = s + v * delta_t + 0.5 * a * delta_t * delta_t;
  st_node->v = v + a * delta_t;
  st_node->cost = cost;
  st_node->cost += weights_.ref_v * std::fabs(v + a * delta_t / 2.0 - ref_v_);
  st_node->cost += weights_.control * fabs(a) * delta_t;
  st_node->parent = this;
  return st_node;
}

void StNode::CalObstacleCost(const double d) {
  constexpr double kMinSafeDistance = 1.0;
  if (d <= kMinSafeDistance) {
    cost += 1e9;
  } else if (d < 2.5) {
    cost += weights_.obstacle * 10 / d;
  }
}
}  // namespace planning
