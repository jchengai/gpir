/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include <memory>

#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gtsam/nonlinear/NonlinearFactor.h"

namespace planning {

class GPObstacleFactor : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  GPObstacleFactor(gtsam::Key key, std::shared_ptr<SignedDistanceField2D> sdf,
                   const double cost, const double epsilon, const double param,
                   const double kappa_r)
      : NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(2, cost), key),
        epsilon_(epsilon),
        param_(param),
        sdf_(sdf),
        kappa_r_(kappa_r) {}
  ~GPObstacleFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector3& x1,
      boost::optional<gtsam::Matrix&> H1 = boost::none) const override;

  gtsam::Matrix23 CheckPointJacobian(const gtsam::Vector3& x,
                                     const double l) const;

 private:
  double epsilon_ = 0.0;
  double param_ = 0.0;

  double kappa_r_ = 0.0;
  double ls_ = 3.0;

  mutable double cos_theta_ = 0.0;
  mutable double sin_theta_ = 0.0;

  std::shared_ptr<SignedDistanceField2D> sdf_;
};
}  // namespace planning
