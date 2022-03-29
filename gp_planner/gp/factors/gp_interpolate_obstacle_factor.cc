/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/factors/gp_interpolate_obstacle_factor.h"

#include "gp_planner/gp/utils/gp_utils.h"

namespace planning {

using gtsam::Vector2;
using gtsam::Vector3;

gtsam::Matrix23 GPInterpolateObstacleFactor::CheckPointJacobian(
    const gtsam::Vector3& x) const {
  gtsam::Matrix23 jacobian = gtsam::Matrix23::Zero();

  const double one_minus_kappar_d = 1 - kappa_r_ * x(0);
  const double denominator =
      1 / (one_minus_kappar_d * one_minus_kappar_d + x(1) * x(1));

  jacobian(0, 0) = ls_ * kappa_r_ * x(1) * sin_theta_ * denominator;
  jacobian(0, 1) = -ls_ * one_minus_kappar_d * sin_theta_ * denominator;
  jacobian(1, 0) = 1 - ls_ * kappa_r_ * x(1) * cos_theta_ * denominator;
  jacobian(1, 1) = ls_ * one_minus_kappar_d * cos_theta_ * denominator;

  return jacobian;
}

gtsam::Vector GPInterpolateObstacleFactor::evaluateError(
    const Vector3& x1, const Vector3& x2, boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const {
  static gtsam::Matrix23 J_x =
      (gtsam::Matrix(2, 3) << 0, 0, 0, 1, 0, 0).finished();
  gtsam::Matrix33 J_x1, J_x2;
  gtsam::Matrix12 J_err1, J_err2;
  gtsam::Matrix23 H;

  Vector3 x_inter = gp_interpolator_.Interpolate(x1, x2, J_x1, J_x2);

  const double theta = std::atan2(x_inter(1), 1 - kappa_r_ * x_inter(0));
  sin_theta_ = std::sin(theta);
  cos_theta_ = std::cos(theta);

  Vector2 point1(param_, x_inter(0));
  Vector2 point2(param_ + ls_ * cos_theta_, x_inter(0) + ls_ * sin_theta_);

  double err1 = GPUtils::HingeLoss2(point1, *sdf_, epsilon_, J_err1);
  double err2 = GPUtils::HingeLoss2(point2, *sdf_, epsilon_, J_err2);

  if (H1 || H2) {
    H.row(0) = J_err1 * J_x;
    H.row(1) = J_err2 * CheckPointJacobian(x_inter);

    *H1 = H * J_x1;
    *H2 = H * J_x2;
  }

  return gtsam::Vector2(err1, err2);
}
}  // namespace planning
