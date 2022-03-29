/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/factors/gp_interpolate_kappa_limit_factor.h"

#include "gp_planner/gp/utils/curvature_utils.h"
#include "gp_planner/gp/utils/gp_utils.h"

namespace planning {

gtsam::Vector GPInterpolateKappaLimitFactor::evaluateError(
    const gtsam::Vector3& x1, const gtsam::Vector3& x2,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const {
  gtsam::Vector3 x;
  double gradient = 0.0, error = 0.0;
  if (H1 || H2) {
    gtsam::Matrix J_err;
    gtsam::Matrix33 J_x1, J_x2;
    x = gp_interpolator_.Interpolate(x1, x2, J_x1, J_x2);
    double kappa =
        CurvatureUtils::GetKappaAndJacobian(x, kappa_r_, dkappa_r_, J_err);
    error = penalty_.EvaluateHinge(kappa, &gradient);
    J_err *= gradient;
    (*H1) = J_err * J_x1;
    (*H2) = J_err * J_x2;
  } else {
    x = gp_interpolator_.Interpolate(x1, x2);
    double kappa = CurvatureUtils::GetKappaAndJacobian(x, kappa_r_, dkappa_r_);
    error = penalty_.EvaluateHinge(kappa, &gradient);
  }

  return gtsam::Vector1(error);
}
}  // namespace planning
