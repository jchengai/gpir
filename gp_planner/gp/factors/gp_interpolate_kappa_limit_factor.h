/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include <memory>

#include "gp_planner/gp/utils/penalty_function.h"
#include "gp_planner/gp/interpolator/gp_interpolator.h"
#include "gp_planner/gp/utils/bounded_penalty_function.h"
#include "gtsam/nonlinear/NonlinearFactor.h"

namespace planning {

class GPInterpolateKappaLimitFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  GPInterpolateKappaLimitFactor(gtsam::Key key1, gtsam::Key key2,
                                const double cost, const double qc,
                                const double interval, const double tau,
                                double kappa_r, const double dkappa_r,
                                const double kappa_limit)
      : NoiseModelFactor2(gtsam::noiseModel::Isotropic::Sigma(1, cost), key1,
                          key2),
        kappa_r_(kappa_r),
        dkappa_r_(dkappa_r),
        kappa_limit_(kappa_limit),
        gp_interpolator_(qc, interval, tau),
        penalty_(kappa_limit) {}
  ~GPInterpolateKappaLimitFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector3& x1, const gtsam::Vector3& x2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

 private:
  double kappa_r_ = 0.0;
  double dkappa_r_ = 0.0;
  double kappa_limit_ = 0.0;

  PenaltyFunction penalty_;
  GPInterpolator gp_interpolator_;
};
}  // namespace planning
