/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/factors/gp_obstacle_factor.h"

#include "gp_planner/gp/utils/gp_utils.h"

namespace planning {

gtsam::Matrix23 GPObstacleFactor::CheckPointJacobian(const gtsam::Vector3& x,
                                                     const double l) const {
   gtsam::Matrix23 jacobian = gtsam::Matrix23::Zero();

  const double one_minus_kappar_d = 1 - kappa_r_ * x(0);
  const double denominator =
      1 / (one_minus_kappar_d * one_minus_kappar_d + x(1) * x(1));

  jacobian(0, 0) = l * kappa_r_ * x(1) * sin_theta_ * denominator;
  jacobian(0, 1) = -l * one_minus_kappar_d * sin_theta_ * denominator;
  jacobian(1, 0) = 1 - l * kappa_r_ * x(1) * cos_theta_ * denominator;
  jacobian(1, 1) = l * one_minus_kappar_d * cos_theta_ * denominator;

  return jacobian;
}

gtsam::Vector GPObstacleFactor::evaluateError(
    const gtsam::Vector3& x1, boost::optional<gtsam::Matrix&> H1) const {
  static gtsam::Matrix23 J_x =
      (gtsam::Matrix(2, 3) << 0, 0, 0, 1, 0, 0).finished();

  const double theta = std::atan2(x1(1), 1 - kappa_r_ * x1(0));
  sin_theta_ = std::sin(theta);
  cos_theta_ = std::cos(theta);

  gtsam::Matrix12 J_err1, J_err2;
  gtsam::Vector2 point1(param_ + ls_ / 2 * cos_theta_,
                        x1(0) + ls_ / 2 * sin_theta_);
  gtsam::Vector2 point2(param_ + ls_ * cos_theta_, x1(0) + ls_ * sin_theta_);

  const double err1 = GPUtils::HingeLoss2(point1, *sdf_, epsilon_, J_err1);
  const double err2 = GPUtils::HingeLoss2(point2, *sdf_, epsilon_, J_err2);

  if (H1) {
    *H1 = gtsam::Matrix23::Zero();
    H1->row(0) = J_err1 * CheckPointJacobian(x1, ls_ / 2);
    H1->row(1) = J_err2 * CheckPointJacobian(x1, ls_);
  }
  return gtsam::Vector2(err1, err2);
}
}  // namespace planning
