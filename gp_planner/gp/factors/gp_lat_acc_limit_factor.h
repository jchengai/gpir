/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/interpolator/gp_interpolator.h"
#include "gp_planner/gp/utils/penalty_function.h"
#include "gtsam/nonlinear/NonlinearFactor.h"

namespace planning {

class GPLatAccLimitFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  GPLatAccLimitFactor(gtsam::Key key1, gtsam::Key key2, const double qc,
                      const double interval, const double tau,
                      const double s_dot, const double s_dot_dot,
                      const double max_lat_acc, const double cost)
      : NoiseModelFactor2(gtsam::noiseModel::Isotropic::Sigma(1, cost), key1,
                          key2),
        s_dot_sqr_(s_dot * s_dot),
        s_dot_dot_(s_dot_dot),
        max_lat_acc_(std::fabs(max_lat_acc)),
        penalty_(std::fabs(max_lat_acc)),
        gp_interpolator_(qc, interval, tau) {}
  ~GPLatAccLimitFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector3& d1, const gtsam::Vector3& d2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

 private:
  double s_dot_sqr_;
  double s_dot_dot_;
  double max_lat_acc_;

  PenaltyFunction penalty_;
  GPInterpolator gp_interpolator_;
};
}  // namespace planning