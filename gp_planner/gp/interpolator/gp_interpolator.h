/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include "gp_planner/gp/model/white_noise_on_acceleration_model_1d.h"
#include "gp_planner/gp/model/white_noise_on_jerk_model_1d.h"
#include "gtsam/base/Matrix.h"

namespace planning {

class GPInterpolator {
 public:
  GPInterpolator() = default;
  GPInterpolator(const double qc, const double interval, const double tau) {
    WhiteNoiseOnJerkModel1D::LambdaAndPsi(qc, interval, tau, &lambda_, &psi_);
  }
  GPInterpolator(const double interval, const double qc)
      : interval_(interval), qc_(qc) {}
  ~GPInterpolator() = default;

  static gtsam::Vector3 Interpolate(const gtsam::Vector3& x1,
                                    const gtsam::Vector3& x2, const double qc,
                                    const double interval, const double tau) {
    gtsam::Matrix33 lambda, psi;
    WhiteNoiseOnJerkModel1D::LambdaAndPsi(qc, interval, tau, &lambda, &psi);
    return lambda * x1 + psi * x2;
  }

  static gtsam::Vector2 Interpolate(const gtsam::Vector2& x1,
                                    const gtsam::Vector2& x2, const double qc,
                                    const double interval, const double tau) {
    gtsam::Matrix22 lambda, psi;
    GPConstVelocityModel1D::LambdaAndPsi(qc, interval, tau, &lambda, &psi);
    return lambda * x1 + psi * x2;
  }

  inline gtsam::Vector3 Interpolate(
      const gtsam::Vector3& x1, const gtsam::Vector3& x2,
      gtsam::OptionalJacobian<3, 3> H1 = boost::none,
      gtsam::OptionalJacobian<3, 3> H2 = boost::none) const {
    if (H1) *H1 = lambda_;
    if (H2) *H2 = psi_;
    return lambda_ * x1 + psi_ * x2;
  }

  void Interpolate(const gtsam::Vector3& x1, const gtsam::Vector3& x2,
                   const double tau, gtsam::Vector3* res) const {
    gtsam::Matrix33 lambda, psi;
    WhiteNoiseOnJerkModel1D::LambdaAndPsi(qc_, interval_, tau, &lambda, &psi);
    (*res) = lambda * x1 + psi * x2;
  }

  inline const gtsam::Matrix33& Lambda() const { return lambda_; }
  inline const gtsam::Matrix33& Psi() const { return psi_; }

 private:
  double interval_ = 0.0;
  double qc_ = 0.0;
  gtsam::Matrix33 lambda_;
  gtsam::Matrix33 psi_;
};
}  // namespace planning