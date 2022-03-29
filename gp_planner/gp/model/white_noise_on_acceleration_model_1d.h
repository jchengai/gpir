/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <Eigen/Core>

namespace planning {

class GPConstVelocityModel1D {
 public:
  static inline Eigen::Matrix2d Q(const double Qc, const double tau) {
    static Eigen::Matrix2d q;
    constexpr double one_third = 1.0 / 3.0;
    q(0, 0) = one_third * std::pow(tau, 3) * Qc;
    q(0, 1) = 0.5 * std::pow(tau, 2) * Qc;
    q(1, 0) = q(0, 1);
    q(1, 1) = tau * Qc;
    return q;
  }

  static inline Eigen::Matrix2d Phi(double tau) {
    static Eigen::Matrix2d phi = Eigen::Matrix2d::Identity();
    phi(0, 1) = tau;
    return phi;
  }

  static inline void LambdaAndPsi(const double qc, const double delta,
                                  const double tau, Eigen::Matrix2d* lambda,
                                  Eigen::Matrix2d* psi) {
    *psi = Q(qc, tau) * (Phi(delta - tau).transpose()) * QInverse(qc, delta);
    *lambda = Phi(tau) - (*psi) * Phi(delta);
  }

 private:
  static inline Eigen::Matrix2d QInverse(const double Qc, const double tau) {
    static Eigen::Matrix2d q_inv;
    const double qc_inv = 1.0 / Qc;
    q_inv(0, 0) = 12 * std::pow(tau, -3) * qc_inv;
    q_inv(0, 1) = -6 * std::pow(tau, -2) * qc_inv;
    q_inv(1, 0) = q_inv(0, 1);
    q_inv(1, 1) = 4 / tau * qc_inv;
    return q_inv;
  }
};
}  // namespace planning