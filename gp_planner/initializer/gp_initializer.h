/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include <Eigen/Core>
#include <vector>

#include "common/base/type.h"

#pragma once

namespace planning {

class GPInitializer {
 public:
  GPInitializer() = default;
  ~GPInitializer() = default;

  void SetBoundary(
      std::vector<std::vector<std::pair<double, double>>> boundary);

  bool GenerateInitialPath(const Eigen::Vector3d& x0, const Eigen::Vector3d& xn,
                           const std::vector<double> s_refs,
                           const std::vector<double>& obstacle_location_hint,
                           const std::vector<double>& lb,
                           std::vector<double>& ub, vector_Eigen3d* result);

 private:
  std::vector<std::pair<double, double>> boundary_;
};
}  // namespace planning
