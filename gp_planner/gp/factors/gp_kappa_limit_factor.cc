/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/factors/gp_kappa_limit_factor.h"

#include "gp_planner/gp/utils/curvature_utils.h"
#include "gp_planner/gp/utils/gp_utils.h"

namespace planning {

gtsam::Vector GPKappaLimitFactor::evaluateError(
    const gtsam::Vector3& x, boost::optional<gtsam::Matrix&> H) const {
  double gradient = 0.0, kappa = 0.0;
  if (H) {
    kappa = CurvatureUtils::GetKappaAndJacobian(x, kappa_r_, dkappa_r_, H);
  } else {
    kappa = CurvatureUtils::GetKappaAndJacobian(x, kappa_r_, dkappa_r_);
  }
  double error = penalty_.EvaluateHinge(kappa, &gradient);
  if (H) (*H) *= gradient;
  return gtsam::Vector1(error);
}
}  // namespace planning
