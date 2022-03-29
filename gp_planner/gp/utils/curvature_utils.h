/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include "gtsam/base/Matrix.h"

namespace planning {

class CurvatureUtils {
 public:
  static double GetKappaAndJacobian(
      const gtsam::Vector3& d, const double kappa_r, const double dkappa_r,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H = boost::none) {
    const double one_minus_kappar_d = 1 - kappa_r * d(0);
    const double one_minus_kappar_d_inv = 1.0 / one_minus_kappar_d;
    const double theta = std::atan2(d(1), one_minus_kappar_d);

    const double tan_theta = d(1) / one_minus_kappar_d;
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    const double cos_theta_sqr = cos_theta * cos_theta;

    const double kappa =
        ((d(2) - (dkappa_r * d(0) + kappa_r * d(1)) * tan_theta) *
             cos_theta_sqr * one_minus_kappar_d_inv +
         kappa_r) *
        cos_theta * one_minus_kappar_d_inv;

    if (H) {
      std::array<double, 2> partial_theta;
      std::array<double, 3> partial_f;

      const double denominator =
          1.0 / (one_minus_kappar_d * one_minus_kappar_d + d(1) * d(1));
      partial_theta[0] = -kappa_r * d(1) * denominator;
      partial_theta[1] = one_minus_kappar_d * denominator;

      const double tmp0 = -3 * cos_theta_sqr * sin_theta;
      const double tmp1 = (dkappa_r * d(0) + kappa_r * d(1)) * cos_theta *
                          (1 - 3 * sin_theta * sin_theta);
      partial_f[0] = (tmp0 - dkappa_r * tmp1) * partial_theta[0] -
                     dkappa_r * sin_theta * cos_theta_sqr;
      partial_f[1] = (tmp0 - kappa_r * tmp1) * partial_theta[1] -
                     kappa_r * sin_theta * cos_theta_sqr;
      partial_f[2] = cos_theta_sqr * cos_theta;

      const double f = (d(2) - (dkappa_r * d(0) + kappa_r * d(1)) * tan_theta) *
                       cos_theta * cos_theta_sqr;

      *H = gtsam::Matrix13::Zero();
      (*H)(0, 0) =
          (partial_f[0] * one_minus_kappar_d +
           2 * kappa_r * f * one_minus_kappar_d) *
              std::pow(one_minus_kappar_d_inv, 4) +
          (-kappa_r * cos_theta_sqr * partial_theta[0] * one_minus_kappar_d +
           kappa_r * kappa_r * cos_theta) *
              one_minus_kappar_d_inv;
      (*H)(0, 1) =
          partial_f[1] * one_minus_kappar_d_inv -
          kappa_r * sin_theta * partial_theta[1] * one_minus_kappar_d_inv;
      (*H)(0, 2) =
          partial_f[2] * one_minus_kappar_d_inv * one_minus_kappar_d_inv;
    }

    return kappa;
  }
};

}  // namespace planning
