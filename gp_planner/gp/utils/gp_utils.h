/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gtsam/base/Matrix.h"

namespace planning {
class GPUtils {
 public:
  static double HingeLoss(const gtsam::Vector2& point,
                          const SignedDistanceField2D& sdf, const double eps,
                          gtsam::OptionalJacobian<1, 2> H_point = boost::none);
  static double HingeLoss2(const gtsam::Vector2& point,
                           const SignedDistanceField2D& sdf, const double eps,
                           gtsam::OptionalJacobian<1, 2> H_point = boost::none);

  static double HingeKappaLimitLoss(
      const gtsam::Vector3& x, const double kappa_r, const double dkappa_r,
      const double kappa_limit,
      gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H = boost::none);
};
}  // namespace planning