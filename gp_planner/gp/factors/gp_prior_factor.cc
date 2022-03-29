/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/gp/factors/gp_prior_factor.h"

namespace planning {

using gtsam::Matrix;

gtsam::Vector GPPriorFactor::evaluateError(
    const gtsam::Vector3& x1, const gtsam::Vector3& x2,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const {
  if (H1) *H1 = phi_;
  if (H2) *H2 = -gtsam::Matrix33::Identity();
  return phi_ * x1 - x2;
}

}  // namespace planning
