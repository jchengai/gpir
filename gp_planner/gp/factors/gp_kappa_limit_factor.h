/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include <memory>

#include "gp_planner/gp/utils/bounded_penalty_function.h"
#include "gp_planner/gp/utils/penalty_function.h"
#include "gtsam/nonlinear/NonlinearFactor.h"

namespace planning {

class GPKappaLimitFactor : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  GPKappaLimitFactor(gtsam::Key key, const double cost, double kappa_r,
                     const double dkappa_r, const double kappa_limit,
                     const double s)
      : NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(1, cost), key),
        kappa_r_(kappa_r),
        dkappa_r_(dkappa_r),
        kappa_limit_(kappa_limit),
        s_(s),
        penalty_(kappa_limit) {}
  ~GPKappaLimitFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector3& x,
      boost::optional<gtsam::Matrix&> H = boost::none) const override;

 private:
  double kappa_r_ = 0.0;
  double dkappa_r_ = 0.0;
  double kappa_limit_ = 0.0;
  double s_ = 0.0;

  PenaltyFunction penalty_;
};
}  // namespace planning
