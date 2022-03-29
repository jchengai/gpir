/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include "gp_planner/gp/model/white_noise_on_jerk_model_1d.h"
#include "gtsam/nonlinear/NonlinearFactor.h"

namespace planning {
class GPPriorFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  GPPriorFactor(gtsam::Key key1, gtsam::Key key2, const double delta,
                const double Qc)
      : NoiseModelFactor2(gtsam::noiseModel::Gaussian::Covariance(
                              WhiteNoiseOnJerkModel1D::Q(Qc, delta)),
                          key1, key2),
        delta_(delta),
        phi_(WhiteNoiseOnJerkModel1D::Phi(delta)){};
  ~GPPriorFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector3& x1, const gtsam::Vector3& x2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

 private:
  double delta_ = 0.0;
  gtsam::Matrix33 phi_;
};
}  // namespace planning