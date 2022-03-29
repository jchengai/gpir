/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <Eigen/Core>

namespace common {
struct State {
  double s = 0.0;
  double stamp = 0.0;
  double heading = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double jerk = 0.0;
  double kappa = 0.0;
  double steer = 0.0;
  Eigen::Vector2d position;
  Eigen::Vector3d debug;
  Eigen::Vector3d frenet_d;
  Eigen::Vector3d frenet_s;

  std::string DebugString() const {
    std::ostringstream os;
    os << "stamp: " << stamp << ", heading: " << heading
       << ", velocity: " << velocity << ", acceleration: " << acceleration
       << " kappa: " << kappa << ", pos (" << position.x() << ", "
       << position.y() << ")";
    return os.str();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace common
