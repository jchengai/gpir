
/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include <memory>

#include "gp_planner/gp/interpolator/gp_interpolator.h"
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gtsam/nonlinear/NonlinearFactor.h"

namespace planning {

class GPInterpolateObstacleFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  GPInterpolateObstacleFactor(gtsam::Key key1, gtsam::Key key2,
                              std::shared_ptr<SignedDistanceField2D> sdf,
                              const double cost, const double epsilon,
                              const double param_start, const double qc,
                              const double interval, const double tau,
                              const double kappa_r)
      : NoiseModelFactor2(gtsam::noiseModel::Isotropic::Sigma(2, cost), key1,
                          key2),
        sdf_(sdf),
        epsilon_(epsilon),
        param_(param_start + tau),
        kappa_r_(kappa_r),
        gp_interpolator_(qc, interval, tau) {}
  ~GPInterpolateObstacleFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector3& x1, const gtsam::Vector3& x2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

  gtsam::Matrix23 CheckPointJacobian(const gtsam::Vector3& x) const;

 private:
  double epsilon_ = 0.0;
  double param_ = 0.0;

  double kappa_r_ = 0;
  double ls_ = 3.0;

  mutable double cos_theta_ = 0.0;
  mutable double sin_theta_ = 0.0;

  GPInterpolator gp_interpolator_;
  std::shared_ptr<SignedDistanceField2D> sdf_;
};
}  // namespace planning