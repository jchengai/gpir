/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/utils/gp_utils.h"

namespace planning {

double GPUtils::HingeLoss(const gtsam::Vector2& point,
                          const SignedDistanceField2D& sdf, const double eps,
                          gtsam::OptionalJacobian<1, 2> H_point) {
  gtsam::Vector2 grad;
  const double signed_distance = sdf.SignedDistance(point, &grad);

  if (signed_distance > eps) {
    if (H_point) *H_point = gtsam::Matrix12::Zero();
    return 0.0;
  } else {
    if (H_point) *H_point = -grad.transpose();
    return eps - signed_distance;
  }
}

double GPUtils::HingeLoss2(const gtsam::Vector2& point,
                           const SignedDistanceField2D& sdf, const double eps,
                           gtsam::OptionalJacobian<1, 2> H_point) {
  gtsam::Vector2 grad;
  const double signed_distance = sdf.SignedDistance(point, &grad);
  double error = eps - signed_distance;

  if (error < 0) {
    if (H_point) *H_point = gtsam::Matrix12::Zero();
    return 0.0;
  } else if (0 < error && error <= eps) {
    if (H_point) (*H_point) = -3 * error * error * grad.transpose();
    return error * error * error;
  } else if (error > eps) {
    if (H_point)
      (*H_point) = -(6 * eps * error - 3 * eps * eps) * grad.transpose();
    return 3 * eps * error * error - 3 * eps * eps * error + eps * eps * eps;
  }
}

double GPUtils::HingeKappaLimitLoss(
    const gtsam::Vector3& x, const double kappa_r, const double dkappa_r,
    const double kappa_limit,
    gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H) {
  const double one_minus_kappar_d = 1 - kappa_r * x(0);
  const double one_minus_kappar_d_inv = 1.0 / one_minus_kappar_d;
  const double theta = std::atan2(x(1), one_minus_kappar_d);

  const double tan_theta = x(1) / one_minus_kappar_d;
  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);
  const double cos_thete_sqr = cos_theta * cos_theta;

  const double kappa =
      ((x(2) - (dkappa_r * x(0) + kappa_r * x(1)) * tan_theta) * cos_thete_sqr *
           one_minus_kappar_d_inv +
       kappa_r) *
      cos_theta * one_minus_kappar_d_inv;

  if (std::fabs(kappa) < kappa_limit) {
    if (H) *H = gtsam::Matrix13::Zero();
    return 0.0;
  }

  if (H) {
    std::array<double, 2> partial_theta;
    std::array<double, 3> partial_f;

    const double denominator =
        1.0 / (one_minus_kappar_d * one_minus_kappar_d + x(1) * x(1));
    partial_theta[0] = -kappa_r * x(1) * denominator;
    partial_theta[1] = one_minus_kappar_d * denominator;

    const double tmp0 = -3 * cos_thete_sqr * sin_theta;
    const double tmp1 = (dkappa_r * x(0) + kappa_r * x(1)) * cos_theta *
                        (1 - 3 * sin_theta * sin_theta);
    partial_f[0] = (tmp0 - dkappa_r * tmp1) * partial_theta[0] -
                   dkappa_r * sin_theta * cos_thete_sqr;
    partial_f[1] = (tmp0 - kappa_r * tmp1) * partial_theta[1] -
                   kappa_r * sin_theta * cos_thete_sqr;
    partial_f[2] = cos_thete_sqr * cos_theta;

    const double f = (x(2) - (dkappa_r * x(0) + kappa_r * x(1)) * tan_theta) *
                     cos_theta * cos_thete_sqr;

    *H = gtsam::Matrix13::Zero();
    (*H)(0, 0) =
        (partial_f[0] * one_minus_kappar_d +
         2 * kappa_r * f * one_minus_kappar_d) *
            std::pow(one_minus_kappar_d_inv, 4) +
        (-kappa_r * cos_thete_sqr * partial_theta[0] * one_minus_kappar_d +
         kappa_r * kappa_r * cos_theta) *
            one_minus_kappar_d_inv;
    (*H)(0, 1) =
        partial_f[1] * one_minus_kappar_d_inv -
        kappa_r * sin_theta * partial_theta[1] * one_minus_kappar_d_inv;
    (*H)(0, 2) = partial_f[2] * one_minus_kappar_d_inv * one_minus_kappar_d_inv;
  }
  if (kappa > 0) {
    return kappa - kappa_limit;
  } else {
    if (H) *H = -(*H);
    return kappa_limit - kappa;
  }
}

}  // namespace planning
